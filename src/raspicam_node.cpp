/*
Copyright (c) 2013, Broadcom Europe Ltd
Copyright (c) 2013, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file RaspiVid.c
 * Command line program to capture a camera video stream and encode it to file.
 * Also optionally display a preview/viewfinder of current camera input.
 *
 * \date 28th Feb 2013
 * \Author: James Hughes
 *
 * Description
 *
 * 3 components are created; camera, preview and video encoder.
 * Camera component has three ports, preview, video and stills.
 * This program connects preview and stills to the preview and video
 * encoder. Using mmal we don't need to worry about buffers between these
 * components, but we do need to handle buffers from the encoder, which
 * are simply written straight to the file in the requisite buffer callback.
 *
 * We use the RaspiCamControl code to handle the specific camera settings.
 * We use the RaspiPreview code to handle the (generic) preview window
 */

// We use some GNU extensions (basename)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#define VCOS_ALWAYS_WANT_LOGGING

#define VERSION_STRING "v1.2"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"

#include "RaspiCamControl.h"
#include "RaspiCLI.h"


#include <semaphore.h>

/// Camera number to use - we only have one camera, indexed from 0.
#define CAMERA_NUMBER 0

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Video format information
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3


/// Interval at which we check for an failure abort during capture



int mmal_status_to_int(MMAL_STATUS_T status);

/** Structure containing all state information for the current run
 */
typedef struct
{
   int isInit;
   int width;                          /// Requested width of image
   int height;                         /// requested height of image
   int framerate;                      /// Requested frame rate (fps)
   int quality;
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
   MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

   MMAL_POOL_T *video_pool; /// Pointer to the pool of buffers used by encoder output port
   MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port
   ros::Publisher *image_pub;
} RASPIVID_STATE;

RASPIVID_STATE state_srv;
ros::Publisher image_pub;
ros::Publisher camera_info_pub;
sensor_msgs::CameraInfo c_info;
std::string tf_prefix;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
   unsigned char *buffer[2];                   /// File handle to write buffer data to.
   RASPIVID_STATE *pstate;              /// pointer to our state in case required in callback
   int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
   int frame;
   int id;
} PORT_USERDATA;

static void display_valid_parameters(char *app_name);




/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void get_status(RASPIVID_STATE *state)
{
   int temp;
   std::string str;
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   // Default everything to zero
   memset(state, 0, sizeof(RASPIVID_STATE));

   if (ros::param::get("~width", temp )){
	if(temp > 0 && temp <= 1920)	
		state->width = temp;
	else	state->width = 640;
   }else{
	state->width = 640;
	ros::param::set("~width", 640);
   }

   if (ros::param::get("~height", temp )){
	if(temp > 0 && temp <= 1080)	
		state->height = temp;
	else	state->height = 480;
   }else{
	state->height = 480;
	ros::param::set("~height", 480);
   }

   if (ros::param::get("~quality", temp )){
	if(temp > 0 && temp <= 100)
		state->quality = temp;
	else	state->quality = 80;
   }else{
	state->quality = 80;
	ros::param::set("~quality", 80);
   }

   if (ros::param::get("~framerate", temp )){
	if(temp > 0 && temp <= 90)
		state->framerate = temp;
	else	state->framerate = 30;
   }else{
	state->framerate = 30;
	ros::param::set("~framerate", 30);
   }

   if (ros::param::get("~tf_prefix",  str)){
	tf_prefix = str;
   }else{
	tf_prefix = "";
	ros::param::set("~tf_prefix", "");
   }

   state->isInit = 0;

   // Setup preview window defaults
   //raspipreview_set_defaults(&state->preview_parameters);

   // Set up the camera_parameters to default
   raspicamcontrol_set_defaults(&state->camera_parameters);
}






/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;
   int complete = 0;

   // We pass our file handle and other stuff in via the userdata field.

   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
   if (pData && pData->pstate->isInit)
   {
      int bytes_written = buffer->length;
      if (buffer->length)
      {
         mmal_buffer_header_mem_lock(buffer);
         memcpy(&(pData->buffer[pData->frame & 1][pData->id]), buffer->data, buffer->length);
		 pData->id += bytes_written;
         mmal_buffer_header_mem_unlock(buffer);
      }

      if (bytes_written != buffer->length)
      {
         vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
         pData->abort = 1;
      }
      if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
         complete = 1;

	if (complete){
		sensor_msgs::CompressedImage msg;
		msg.header.seq = pData->frame;
		msg.header.frame_id = tf_prefix;
		msg.header.frame_id.append("/camera");
		msg.header.stamp = ros::Time::now();
		msg.format = "jpg";
		msg.data.insert( msg.data.end(), pData->buffer[pData->frame & 1], &(pData->buffer[pData->frame & 1][pData->id]) );
		image_pub.publish(msg);
		c_info.header.seq = pData->frame;
		c_info.header.stamp = msg.header.stamp;
		c_info.header.frame_id = msg.header.frame_id;
		camera_info_pub.publish(c_info);
		pData->frame++;
		pData->id = 0;		
	}
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }
}



/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state)
{
   MMAL_COMPONENT_T *camera = 0;
   MMAL_ES_FORMAT_T *format;
   MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
   MMAL_STATUS_T status;

   /* Create the component */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create camera component");
      goto error;
   }

   if (!camera->output_num)
   {
      vcos_log_error("Camera doesn't have output ports");
      goto error;
   }

   video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
   still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];


   //  set up the camera configuration
   {
      MMAL_PARAMETER_CAMERA_CONFIG_T cam_config;
      cam_config.hdr.id = MMAL_PARAMETER_CAMERA_CONFIG;
      cam_config.hdr.size = sizeof(cam_config);
      cam_config.max_stills_w = state->width;
      cam_config.max_stills_h = state->height;
      cam_config.stills_yuv422 = 0;
      cam_config.one_shot_stills = 0;
      cam_config.max_preview_video_w = state->width;
      cam_config.max_preview_video_h = state->height;
      cam_config.num_preview_video_frames = 3;
      cam_config.stills_capture_circular_buffer_height = 0;
      cam_config.fast_preview_resume = 0;
      cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC;

      mmal_port_parameter_set(camera->control, &cam_config.hdr);
   }

   // Now set up the port formats

   // Set the encode format on the video  port

   format = video_port->format;
   format->encoding_variant = MMAL_ENCODING_I420;

   format->encoding = MMAL_ENCODING_I420;
   format->es->video.width = state->width;
   format->es->video.height = state->height;
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->width;
   format->es->video.crop.height = state->height;
   format->es->video.frame_rate.num = state->framerate;
   format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

   status = mmal_port_format_commit(video_port);

   if (status)
   {
      vcos_log_error("camera video format couldn't be set");
      goto error;
   }

   // Ensure there are enough buffers to avoid dropping frames
   if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   // Set the encode format on the still  port

   format = still_port->format;

   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;

   format->es->video.width = state->width;
   format->es->video.height = state->height;
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->width;
   format->es->video.crop.height = state->height;
   format->es->video.frame_rate.num = 1;
   format->es->video.frame_rate.den = 1;

   status = mmal_port_format_commit(still_port);

   if (status)
   {
      vcos_log_error("camera still format couldn't be set");
      goto error;
   }
   
	video_port->buffer_num = video_port->buffer_num_recommended;
   /* Ensure there are enough buffers to avoid dropping frames */
   if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   /* Enable component */
   status = mmal_component_enable(camera);

   if (status)
   {
      vcos_log_error("camera component couldn't be enabled");
      goto error;
   }

   raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

   state->camera_component = camera;

   ROS_INFO("Camera component done\n");

   return camera;

error:

   if (camera)
      mmal_component_destroy(camera);

   return 0;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVID_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}

 /**
  * Create the encoder component, set up its ports
  *
  * @param state Pointer to state control struct
  *
  * @return MMAL_SUCCESS if all OK, something else otherwise
  *
  */
 static MMAL_STATUS_T create_encoder_component(RASPIVID_STATE *state)
 {
    MMAL_COMPONENT_T *encoder = 0;
    MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
    MMAL_STATUS_T status;
    MMAL_POOL_T *pool;
 
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);

    if (status != MMAL_SUCCESS)
    {
       vcos_log_error("Unable to create video encoder component");
       goto error;
    }
 
    if (!encoder->input_num || !encoder->output_num)
    {
       status = MMAL_ENOSYS;
       vcos_log_error("Video encoder doesn't have input/output ports");
       goto error;
    }
 
    encoder_input = encoder->input[0];
    encoder_output = encoder->output[0];
 
    // We want same format on input and output
    mmal_format_copy(encoder_output->format, encoder_input->format);
 
    // Only supporting H264 at the moment
    encoder_output->format->encoding = MMAL_ENCODING_JPEG;
 
 
    encoder_output->buffer_size = encoder_output->buffer_size_recommended;
 
    if (encoder_output->buffer_size < encoder_output->buffer_size_min)
       encoder_output->buffer_size = encoder_output->buffer_size_min;
 
    encoder_output->buffer_num = encoder_output->buffer_num_recommended;
 
    if (encoder_output->buffer_num < encoder_output->buffer_num_min)
       encoder_output->buffer_num = encoder_output->buffer_num_min;
 
    // Commit the port changes to the output port
    status = mmal_port_format_commit(encoder_output);
 
    if (status != MMAL_SUCCESS)
    {
       vcos_log_error("Unable to set format on video encoder output port");
       goto error;
    }
 
 // Set the JPEG quality level
   status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, state->quality);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set JPEG quality");
      goto error;
   }

   
    //  Enable component
    status = mmal_component_enable(encoder);
 
    if (status != MMAL_SUCCESS)
    {
       vcos_log_error("Unable to enable video encoder component");
       goto error;
    }
 
    /* Create pool of buffer headers for the output port to consume */
    pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);
 
    if (!pool)
    {
       vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
    }
 
    state->encoder_pool = pool;
    state->encoder_component = encoder;
 
    ROS_INFO("Encoder component done\n");
 
    return status;
 
    error:
    if (encoder)
       mmal_component_destroy(encoder);
 
    return status;
 }

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPIVID_STATE *state)
{
   // Get rid of any port buffers first
   if (state->video_pool)
   {
      mmal_port_pool_destroy(state->encoder_component->output[0], state->video_pool);
   }

   if (state->encoder_component)
   {
      mmal_component_destroy(state->encoder_component);
      state->encoder_component = NULL;
   }
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
   MMAL_STATUS_T status;

   status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

   if (status == MMAL_SUCCESS)
   {
      status =  mmal_connection_enable(*connection);
      if (status != MMAL_SUCCESS)
         mmal_connection_destroy(*connection);
   }

   return status;
}

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port)
{
   if (port && port->is_enabled)
      mmal_port_disable(port);
}

/**
 * Handler for sigint signals
 *
 * @param signal_number ID of incoming signal.
 *
 */
static void signal_handler(int signal_number)
{
   // Going to abort on all signals
   vcos_log_error("Aborting program\n");

   // TODO : Need to close any open stuff...how?

   exit(255);
}

/**
 * init_cam

 */
int init_cam(RASPIVID_STATE *state)
{
   // Our main data storage vessel..
   MMAL_STATUS_T status;
   MMAL_PORT_T *camera_video_port = NULL;
   MMAL_PORT_T *camera_still_port = NULL;
   MMAL_PORT_T *preview_input_port = NULL;
   MMAL_PORT_T *encoder_input_port = NULL;
   MMAL_PORT_T *encoder_output_port = NULL;

   bcm_host_init();
   get_status(state);
   // Register our application with the logging system
   vcos_log_register("RaspiVid", VCOS_LOG_CATEGORY);

   signal(SIGINT, signal_handler);

   // OK, we have a nice set of parameters. Now set up our components
   // We have three components. Camera, Preview and encoder.

   if (!create_camera_component(state))
   {
      ROS_INFO("%s: Failed to create camera component", __func__);
   }
   else if ((status = create_encoder_component(state)) != MMAL_SUCCESS)
   {
      ROS_INFO("%s: Failed to create encode component", __func__);
      destroy_camera_component(state);
   }
   else
   {
      PORT_USERDATA * callback_data_enc = (PORT_USERDATA *) malloc (sizeof(PORT_USERDATA));
      camera_video_port   = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];
      camera_still_port   = state->camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
      encoder_input_port  = state->encoder_component->input[0];
      encoder_output_port = state->encoder_component->output[0];
      status = connect_ports(camera_video_port, encoder_input_port, &state->encoder_connection);
      if (status != MMAL_SUCCESS)
      {
            ROS_INFO("%s: Failed to connect camera video port to encoder input", __func__);
	    return 1;
      }
      callback_data_enc->buffer[0] = (unsigned char *) malloc ( 1024 * 1024 );
      callback_data_enc->buffer[1] = (unsigned char *) malloc ( 1024 * 1024 );
      // Set up our userdata - this is passed though to the callback where we need the information.
      callback_data_enc->pstate = state;
      callback_data_enc->abort = 0;
      callback_data_enc->id = 0;
      callback_data_enc->frame = 0;
      encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *) callback_data_enc;
      PORT_USERDATA *pData = (PORT_USERDATA *)encoder_output_port->userdata;
      // Enable the encoder output port and tell it its callback function
      status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);
      if (status != MMAL_SUCCESS)
      {
         ROS_INFO("Failed to setup encoder output");
         return 1;
      }
      state->isInit = 1;
   }
   return 0;
}


int start_capture(RASPIVID_STATE *state){
	if(!(state->isInit)) init_cam(state);
	MMAL_PORT_T *camera_video_port   = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];
	MMAL_PORT_T *encoder_output_port = state->encoder_component->output[0];
	ROS_INFO("Starting video capture (%d, %d, %d, %d)\n", state->width, state->height, state->quality, state->framerate);

      	if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
      	{
	 	return 1;
      	}
      	// Send all the buffers to the video port
      	{
	 	int num = mmal_queue_length(state->encoder_pool->queue);
	 	int q;
	 	for (q=0;q<num;q++)
	 	{
	      	MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state->encoder_pool->queue);

	      	if (!buffer)
	        	vcos_log_error("Unable to get a required buffer %d from pool queue", q);

	       	if (mmal_port_send_buffer(encoder_output_port, buffer)!= MMAL_SUCCESS)
	           	vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);

	 	}
      	}
	ROS_INFO("Video capture started\n");
	return 0;

}



int close_cam(RASPIVID_STATE *state){
	if(state->isInit){
		state -> isInit = 0;
		MMAL_COMPONENT_T *camera = state->camera_component;
		MMAL_COMPONENT_T *encoder = state->encoder_component;
		MMAL_PORT_T *encoder_output_port = state->encoder_component->output[0];
		MMAL_PORT_T *camera_still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];
		PORT_USERDATA * pData = (PORT_USERDATA *)encoder_output_port->userdata;

		if (camera_still_port && camera_still_port->is_enabled)
			mmal_port_disable(camera_still_port);
		  
		if (encoder->output[0] && encoder->output[0]->is_enabled)
			mmal_port_disable(encoder->output[0]);

		mmal_connection_destroy(state->encoder_connection);

		// Disable components
		if (encoder)
			mmal_component_disable(encoder);
		 
		if (camera)
			mmal_component_disable(camera);
			 
		//Destroy encoder component
		// Get rid of any port buffers first
		if (state->encoder_pool)
		{
			mmal_port_pool_destroy(encoder->output[0], state->encoder_pool);
		}


		free(pData->buffer[0]);
		free(pData->buffer[1]);

		if (encoder)
		{
			mmal_component_destroy(encoder);
			encoder = NULL;
		}
		//destroy camera component
		if (camera)
		{
			mmal_component_destroy(camera);
			camera = NULL;
		}
		ROS_INFO("Video capture stopped\n");
		return 0;
	}else return 1;
}

bool serv_start_cap(	std_srvs::Empty::Request  &req,
			std_srvs::Empty::Response &res )
{
  start_capture(&state_srv);
  return true;
}


bool serv_stop_cap(	std_srvs::Empty::Request  &req,
			std_srvs::Empty::Response &res )
{
  close_cam(&state_srv);
  return true;
}



int main(int argc, char **argv){
   ros::init(argc, argv, "raspicam_node");
   ros::NodeHandle n;
   camera_info_manager::CameraInfoManager c_info_man (n, "camera", "package://raspicam/calibrations/camera.yaml");
   get_status(&state_srv);

   if(!c_info_man.loadCameraInfo ("package://raspicam/calibrations/camera.yaml")){
	ROS_INFO("Calibration file missing. Camera not calibrated");
   }
   else
   {
   	c_info = c_info_man.getCameraInfo ();
	ROS_INFO("Camera successfully calibrated");
   }
   image_pub = n.advertise<sensor_msgs::CompressedImage>("camera/image/compressed", 1);
   camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);
   ros::ServiceServer start_cam = n.advertiseService("camera/start_capture", serv_start_cap);
   ros::ServiceServer stop_cam = n.advertiseService("camera/stop_capture", serv_stop_cap);
   ros::spin();
   close_cam(&state_srv);
   return 0;
}
