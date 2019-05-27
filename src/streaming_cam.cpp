// ROS INCLUDES
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// GSTREAMER INCLUDES
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtsp-server/rtsp-media-factory.h>
#include <gst/app/app.h>


#include <thread>

using namespace ros;
using std::string;



// This struct store the context used by pipeline
typedef struct {
    int width;
    int height;
    GstClockTime timestamp;
    bool clientsConnected;
    // If the pipeline is full ? true : false
    bool full;
    GstElement* appSrc;

    // Parameters

    // The topic from which to take frames
    string topic;

    // Color format
    string color;
    int framerate;

    // high-quality  medium-quality  low-quality
    string profile;
    // Subpixel motion estimation and partition decision quality: (fast - worst) 1 <= x <= 10 (slow - best)
    int subme;
    // Constant quantizer or quality to apply. (high quality) 1 <= x <= 50 (low quality)
    int quantizer;
    // Number of threads used by the encoder (0 = auto)
    int threads;
} Context;

// FUNCTIONS PROTOTYPES

// This function stores the parameters
void store_parameters(Context*);

// This function pushes the frame inside the pipeline
void push_frame(const sensor_msgs::Image::ConstPtr&, const boost::shared_ptr<Context>&);

// This function is called when the first client connects
static void media_configure(GstRTSPMediaFactory*, GstRTSPMedia*, Context*);

// This function is called when the state of pipeline change
static void change_state(GstRTSPMedia*, gint, Context*);

// This function is called when last client disconnects
static void unprepared(GstRTSPMedia*, Context*);

// This function is called when the pipeline emit a signal to request data
static void need_data(GstAppSrc*, guint, Context*);

// This function is called when the pipeline is full
static void too_much_data(GstAppSrc*, Context*);

// This function is call in a separate thread and it starts the server
static void start_server(GMainLoop*);

int main(int argc, char **argv){

    // Init GStreamer
    gst_init(NULL, NULL);

    // Init ROS
    ros::init(argc, argv, "streaming_cam");

    Context* context = new Context();

    // Acquire parameters
    store_parameters(context);

    // Build server RTSP
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    GstRTSPServer* server = gst_rtsp_server_new();
    GstRTSPMountPoints* mountPoints = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();

    // Set up the pipeline
    gst_rtsp_media_factory_set_launch(factory,
            "( appsrc name=appsrc ! videoconvert ! video/x-raw,format=I420 ! "
            "x264enc name=encoder pass=qual ! rtph264pay name=pay0 pt=96 )");

    g_signal_connect(factory, "media-configure", (GCallback)media_configure, context);
    // A single factory (pipeline) shared by all clients
    gst_rtsp_media_factory_set_shared(factory, TRUE);

    gst_rtsp_mount_points_add_factory(mountPoints, "/streaming_cam", factory);

    g_object_unref(mountPoints);

    gst_rtsp_server_attach(server, NULL);

    boost::shared_ptr<Context> ptrContext;
    ptrContext.reset(context);

    NodeHandle node;
    Subscriber subscriber = node.subscribe<sensor_msgs::Image>(context->topic, 1000,
            boost::bind(push_frame, _1, ptrContext));

    std::thread startServer(start_server, loop);

    ros::spin();
}

void store_parameters(Context* parameters){
    ros::NodeHandle nodeHandle("~");

    // Camera paramters
    if(!nodeHandle.getParam("topic", parameters->topic))
        ROS_ERROR("Parameter topic missing");
    nodeHandle.param<string>("color", parameters->color, "RGB");
    if(!nodeHandle.getParam("width", parameters->width))
        ROS_ERROR("Parameter width missing");
    if(!nodeHandle.getParam("height", parameters->height))
        ROS_ERROR("Parameter height missing");

    // Pipeline parameters
    nodeHandle.param<int>("framerate", parameters->framerate, 20);

    // Encoder parameters
    if(!nodeHandle.getParam("profile", parameters->profile))
        ROS_ERROR("Parameter profile missing");

    if(parameters->profile == "high-quality"){
        parameters->subme = 8;
        parameters->quantizer = 10;
        parameters->threads = 0;
    }
    else if(parameters->profile == "medium-quality"){
        parameters->subme = 5;
        parameters->quantizer = 21;
        parameters->threads = 0;
    }
    else if (parameters->profile == "low-quality"){
        parameters->subme = 8;
        parameters->quantizer = 30;
        parameters->threads = 0;
    }
    else if (parameters->profile == "customized"){
        if(!nodeHandle.getParam("subme", parameters->subme))
            ROS_ERROR("Parameter subme missing");

        if(!nodeHandle.getParam("quantizer", parameters->quantizer))
            ROS_ERROR("Parameter quantizer missing");

        if(!nodeHandle.getParam("threads", parameters->threads))
            ROS_ERROR("Parameter threads missing");
    }
    else
        ROS_ERROR("Parameter profile wrong");

}

void push_frame(const sensor_msgs::Image::ConstPtr& image, const boost::shared_ptr<Context>& context){

    if(context->clientsConnected && !context->full){
        GstBuffer *buffer;
        guint size;
        GstFlowReturn ret;
        GstMapInfo map;

        size = context->width * context->height * 3;

        buffer = gst_buffer_new_allocate(NULL, size, NULL);
        gst_buffer_map(buffer, &map, GST_MAP_WRITE);
        memmove(map.data, image->data.data(), gst_buffer_get_size(buffer));

        GST_BUFFER_PTS(buffer) = context->timestamp;
        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, context->framerate);
        context->timestamp += GST_BUFFER_DURATION(buffer);

        g_signal_emit_by_name(context->appSrc, "push-buffer", buffer, &ret);

        gst_buffer_unmap(buffer, &map);
        gst_buffer_unref(buffer);
    }
}

static void media_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, Context* context) {

    g_print("First client connected, pipeline started\n");

    // Connect callback to media signals
    g_signal_connect(media, "new-state", (GCallback)change_state, context);
    g_signal_connect(media, "unprepared", (GCallback)unprepared, context);

    // Get appSrc element
    GstElement* element = gst_rtsp_media_get_element(media);
    GstElement* appSrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "appsrc");

    // Set appSrc parameters
    gst_util_set_object_arg(G_OBJECT(appSrc), "format", "time");

    const string c = context->color;

    // Set appSrc caps
    g_object_set(G_OBJECT(appSrc), "caps",
                gst_caps_new_simple("video/x-raw",
                                    "format", G_TYPE_STRING, context->color.c_str(),
                                    "width", G_TYPE_INT, context->width,
                                    "height", G_TYPE_INT, context->height,
                                    "framerate", GST_TYPE_FRACTION, 1, context->framerate, NULL),
                NULL);


    // Connect appSrc signals to callbacks
    g_signal_connect(appSrc, "need-data", (GCallback)need_data, context);
    g_signal_connect(appSrc, "enough-data", (GCallback)too_much_data, context);

    // Update context
    context->timestamp = 0;
    context->clientsConnected = true;
    context->appSrc = appSrc;

    gst_object_unref(appSrc);

    // Get h264 encoder
    GstElement* encoder = gst_bin_get_by_name_recurse_up(GST_BIN(element), "encoder");

    // Set encoder parameters
    g_object_set(G_OBJECT(encoder), "subme", context->subme, NULL);
    g_object_set(G_OBJECT(encoder), "quantizer", context->quantizer, NULL);
    g_object_set(G_OBJECT(encoder), "threads", context->threads, NULL);

    gst_object_unref(encoder);

    gst_object_unref(element);
}

static void change_state(GstRTSPMedia*, gint state, Context* context) {
    if (state == GST_STATE_NULL) {
        context->clientsConnected = false;
        context->full = false;
    }
}

static void unprepared(GstRTSPMedia*, Context* context) {
    context->clientsConnected = false;
    context->full = false;
}

static void need_data(GstAppSrc*, guint, Context* context){
    context->full = false;
}

static void too_much_data(GstAppSrc*, Context* context) {
    context->full = true;
}

static void start_server(GMainLoop* loop){
    g_print("Stream ready at rtsp://127.0.0.1:8554/streaming_cam\n");
    g_main_loop_run(loop);
}