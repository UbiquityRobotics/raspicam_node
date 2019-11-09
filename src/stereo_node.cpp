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
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#define DBG_MSG(msg)  std::cout << msg << std::endl; 

#ifdef __x86_64__

#include <stdio.h>

int main(int argc, char** argv) {
  (void)fprintf(stderr, "The raspicam_node for the x86/64 architecture is a fake!\n");
  return 1;
}

#endif  // __x86_64__

#ifdef __arm__

// We use some GNU extensions (basename)
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <memory>

#define VCOS_ALWAYS_WANT_LOGGING
#define VERSION_STRING "v1.2"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"

#include "camera_info_manager/camera_info_manager.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "std_srvs/Empty.h"
 #include "raspicam_node/MotionVectors.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
extern "C" {
#include "RaspiCamControl.h"
}
#include <dynamic_reconfigure/server.h>
#include <raspicam_node/CameraConfig.h>

#include "mmal_cxx_helper.h"

static constexpr int IMG_BUFFER_SIZE = 10 * 1024 * 1024;  // 10 MB

// Video format information
static constexpr int VIDEO_FRAME_RATE_DEN = 1;

// Video render needs at least 2 buffers.
static constexpr int VIDEO_OUTPUT_BUFFERS_NUM = 3;

/** Structure containing all state information for the current run
 */
struct RASPIVID_STATE {
  RASPIVID_STATE()
    : camera_component(nullptr)
    , splitter_component(nullptr)
    , splitter_connection(nullptr)
    , splitter_pool(nullptr, mmal::default_delete_pool){}
  bool isInit;
  int width;      /// Requested width of image
  int height;     /// requested height of image
  int framerate;  /// Requested frame rate (fps)
  int quality;

  int camera_id = 0;

  RASPICAM_CAMERA_PARAMETERS camera_parameters;  /// Camera setup parameters

  mmal::component_ptr camera_component;
  mmal::component_ptr splitter_component;

  mmal::connection_ptr splitter_connection;       /// Pointer to camera => splitter
  mmal::pool_ptr splitter_pool;       // Pointer buffer pool used by splitter (raw) output
  // The Updater class advertises to /diagnostics, and has a
  // ~diagnostic_period parameter that says how often the diagnostics
  // should be published.
  diagnostic_updater::Updater updater;
};

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct MMAL_PORT_USERDATA_T {
  MMAL_PORT_USERDATA_T(RASPIVID_STATE& state) : pstate(state){};
  std::unique_ptr<uint8_t[]> buffer[2];  // Memory to write buffer data to.
  RASPIVID_STATE& pstate;                // pointer to our state for use by callback
  bool abort;                            // Set to 1 in callback if an error occurs to attempt to abort
                                         // the capture
  int frame;
  int id;

  int frames_skipped = 0;
} PORT_USERDATA;

using diagnostic_updater::DiagnosedPublisher;
using diagnostic_updater::FrequencyStatusParam;
using diagnostic_updater::TimeStampStatusParam;

// Helper template
template<typename T>
struct DiagnosedMsgPublisher {
  std::unique_ptr<DiagnosedPublisher<T>> pub;
  T msg;
};

static DiagnosedMsgPublisher<sensor_msgs::Image> left_image;
static DiagnosedMsgPublisher<sensor_msgs::Image> right_image;

ros::Publisher left_camera_info_pub;
ros::Publisher right_camera_info_pub;
sensor_msgs::CameraInfo left_info;
sensor_msgs::CameraInfo right_info;
std::string camera_frame_id;
int skip_frames = 0;

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state state structure to assign defaults to
 * @param nh NodeHandle to get params from
 */
void print_image(const sensor_msgs::Image& msg)
{
  std::cout << "header { stamp: { sec: " << msg.header.stamp.sec << ", nsec: " <<  msg.header.stamp.nsec 
      << ", frameid: " << msg.header.frame_id << " }, \nheight: " << msg.height << "\nwidth: " << msg.width 
      << "\nstep: " << msg.step << "\nencoding: " << msg.encoding << "\ndata_size: " << msg.data.size() << std::endl;
}
static void configure_parameters(RASPIVID_STATE& state, ros::NodeHandle& nh) {
  nh.param<int>("width", state.width, 1280);
  nh.param<int>("height", state.height, 720);
  state.height*=2;
  nh.param<int>("quality", state.quality, 80);
  if (state.quality < 0 && state.quality > 100) {
    ROS_WARN("quality: %d is outside valid range 0-100, defaulting to 80", state.quality);
    state.quality = 80;
  }
  nh.param<int>("framerate", state.framerate, 15);
  if (state.framerate < 0 && state.framerate > 90) {
    ROS_WARN("framerate: %d is outside valid range 0-90, defaulting to 30", state.framerate);
    state.framerate = 30;
  }

  nh.param<std::string>("camera_frame_id", camera_frame_id, "stereo");

  nh.param<int>("camera_id", state.camera_id, 0);

  // Set up the camera_parameters to default
  raspicamcontrol_set_defaults(&state.camera_parameters);

  bool temp;
  nh.param<bool>("hFlip", temp, false);
  state.camera_parameters.hflip = temp;  // Hack for bool param => int variable
  nh.param<bool>("vFlip", temp, false);
  state.camera_parameters.vflip = temp;  // Hack for bool param => int variable
  nh.param<int>("shutter_speed", state.camera_parameters.shutter_speed, 0);

  state.isInit = false;
}

/**
 *  buffer header callback function for image encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
void update_image(sensor_msgs::Image& msg, MMAL_BUFFER_HEADER_T* buffer, PORT_USERDATA* pData)
{
  msg.header.frame_id = camera_frame_id;
  msg.header.stamp = ros::Time::now(); //(double(buffer->dts)/1e6);
  msg.encoding = "bgr8";
// #ifdef _DEBUG
//   DBG_MSG("//DEBUG// - buffer->dts:")
//   DBG_MSG( buffer->dts)
//   DBG_MSG(MMAL_TIME_UNKNOWN)
// #endif  // _DEBUG
  msg.is_bigendian = false;
  msg.height = pData->pstate.height/2;
  msg.width = pData->pstate.width;
  msg.step = (pData->pstate.width * 3);
}
static void splitter_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer) {
  // We pass our file handle and other stuff in via the userdata field.
  PORT_USERDATA* pData = port->userdata;
  if (pData && pData->pstate.isInit) {
    size_t bytes_written = buffer->length;
    if (buffer->length) {
      if (pData->id != INT_MAX) {
        if (pData->id + buffer->length > IMG_BUFFER_SIZE) {
          ROS_ERROR("pData->id (%d) + buffer->length (%d) > "
                    "IMG_BUFFER_SIZE (%d), skipping the frame",
                    pData->id, buffer->length, IMG_BUFFER_SIZE);
          pData->id = INT_MAX;  // mark this frame corrupted
        } else {
          mmal_buffer_header_mem_lock(buffer);
          memcpy(&(pData->buffer[pData->frame & 1].get()[pData->id]), buffer->data, buffer->length);
          pData->id += bytes_written;
          mmal_buffer_header_mem_unlock(buffer);
        }
      }
    }

    if (bytes_written != buffer->length) {
      vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
      ROS_ERROR("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
      pData->abort = true;
    }

    int complete = false;
    if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
      complete = true;

    if (complete) {
      if (pData->id != INT_MAX) {
        // ROS_INFO("Frame size %d", pData->id);
        if (skip_frames > 0 && pData->frames_skipped < skip_frames) {
          pData->frames_skipped++;
        } else {
          pData->frames_skipped = 0;
          update_image(left_image.msg, buffer, pData);
          update_image(right_image.msg, buffer, pData);
          auto start = pData->buffer[pData->frame & 1].get();
          auto end = &(pData->buffer[pData->frame & 1].get()[pData->id/2]);
          auto start1 = &(pData->buffer[pData->frame & 1].get()[pData->id/2]);
          auto end1 = &(pData->buffer[pData->frame & 1].get()[pData->id]);
          left_image.msg.data.resize(pData->id/2);
          #ifdef _DEBUG
            DBG_MSG("//DEBUG// - Fill in left image!!!")
          #endif  // _DEBUG
          std::copy(start1, end1, left_image.msg.data.begin());
          #ifdef _DEBUG
            DBG_MSG("//DEBUG// - Success")
          #endif  // _DEBUG
          right_image.msg.data.resize(pData->id/2);
          #ifdef _DEBUG
            DBG_MSG("//DEBUG// - Fill in right image!!!")
          #endif  // _DEBUG
          std::copy(start, end, right_image.msg.data.begin());
          #ifdef _DEBUG
            DBG_MSG("//DEBUG// - Success!!!")
            DBG_MSG("//DEBUG// - Publishing messages!!!")
            print_image(left_image.msg);
            print_image(right_image.msg);
          #endif  // _DEBUG
          
            left_image.pub->publish(left_image.msg);
            right_image.pub->publish(right_image.msg);

          #ifdef _DEBUG
            DBG_MSG("//DEBUG// - Success!!!")
          #endif  // _DEBUG
          left_info.header = left_image.msg.header;
          right_info.header = left_image.msg.header;
          left_camera_info_pub.publish(left_info);
          right_camera_info_pub.publish(right_info);
          
        }
      }
      pData->frame++;
      pData->id = 0;
    }
  }

  // release buffer back to the pool
  mmal_buffer_header_release(buffer);

  // and send one back to the port (if still open)
  if (port->is_enabled) {
    MMAL_STATUS_T status;

    MMAL_BUFFER_HEADER_T* new_buffer = mmal_queue_get(pData->pstate.splitter_pool->queue);

    if (new_buffer)
      status = mmal_port_send_buffer(port, new_buffer);

    if (!new_buffer || status != MMAL_SUCCESS) {
      vcos_log_error("Unable to return a buffer to the splitter port");
      ROS_ERROR("Unable to return a buffer to the splitter port");
    }
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
static MMAL_COMPONENT_T* create_camera_component(RASPIVID_STATE& state) {
  MMAL_COMPONENT_T* camera = 0;
  MMAL_ES_FORMAT_T* format;
  MMAL_PORT_T *video_port = nullptr, *preview_port = nullptr, *still_port = nullptr;
  MMAL_STATUS_T status;

  /* Create the component */
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create camera component");
    ROS_ERROR("Failed to create camera component");
    goto error;
  }

  state.camera_parameters.stereo_mode.mode = stereo_mode_from_string("tb");
  status = static_cast<MMAL_STATUS_T>(raspicamcontrol_set_stereo_mode(camera->output[0], &state.camera_parameters.stereo_mode));
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Could not set stereo mode : error %d", status);
		goto error;
	}
  status = static_cast<MMAL_STATUS_T>(raspicamcontrol_set_stereo_mode(camera->output[1], &state.camera_parameters.stereo_mode));
  
  if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Could not set stereo mode : error %d", status);
		goto error;
	}
  status = static_cast<MMAL_STATUS_T>(raspicamcontrol_set_stereo_mode(camera->output[2], &state.camera_parameters.stereo_mode));

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Could not set stereo mode : error %d", status);
		goto error;
	}
  if (!camera->output_num) {
    vcos_log_error("Camera doesn't have output ports");
    ROS_ERROR("Camera doesn't have output ports");
    goto error;
  }

  video_port = camera->output[mmal::camera_port::video];
  preview_port = camera->output[mmal::camera_port::preview];
  still_port = camera->output[mmal::camera_port::capture];

  //  set up the camera configuration
  {
    MMAL_PARAMETER_CAMERA_CONFIG_T cam_config;
    cam_config.hdr.id = MMAL_PARAMETER_CAMERA_CONFIG;
    cam_config.hdr.size = sizeof(cam_config);
    cam_config.max_stills_w = state.width;
    cam_config.max_stills_h = state.height;
    cam_config.stills_yuv422 = 0;
    cam_config.one_shot_stills = 0;
    cam_config.max_preview_video_w = state.width;
    cam_config.max_preview_video_h = state.height;
    cam_config.num_preview_video_frames = 3;
    cam_config.stills_capture_circular_buffer_height = 0;
    cam_config.fast_preview_resume = 0;
    cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC;

    mmal_port_parameter_set(camera->control, &cam_config.hdr);
  }

  // Select the camera to use
  {
    MMAL_PARAMETER_INT32_T camera_num;
    camera_num.hdr.id = MMAL_PARAMETER_CAMERA_NUM;
    camera_num.hdr.size = sizeof(camera_num);
    camera_num.value = state.camera_id;

    status = mmal_port_parameter_set(camera->control, &camera_num.hdr);
    if (status != MMAL_SUCCESS) {
      ROS_ERROR("Could not select camera : error %d", status);
      goto error;
    }
  }

  // Now set up the port formats

  // Set the encode format on the video  port

  format = video_port->format;
  format->encoding_variant = MMAL_ENCODING_I420;

  format->encoding = MMAL_ENCODING_I420;
  format->es->video.width = state.width;
  format->es->video.height = state.height;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = state.width;
  format->es->video.crop.height = state.height;
  format->es->video.frame_rate.num = state.framerate;
  format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

  status = mmal_port_format_commit(video_port);

  if (status) {
    vcos_log_error("camera video format couldn't be set");
    ROS_ERROR("camera video format couldn't be set");
    goto error;
  }

  // Ensure there are enough buffers to avoid dropping frames
  if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

  // Set the encode format on the preview  port

  format = preview_port->format;
  format->encoding_variant = MMAL_ENCODING_I420;

  format->encoding = MMAL_ENCODING_I420;
  format->es->video.width = state.width;
  format->es->video.height = state.height;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = state.width;
  format->es->video.crop.height = state.height;
  format->es->video.frame_rate.num = state.framerate;
  format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

  status = mmal_port_format_commit(preview_port);

  if (status) {
    vcos_log_error("camera preview format couldn't be set");
    ROS_ERROR("camera preview format couldn't be set");
    goto error;
  }

  // Ensure there are enough buffers to avoid dropping frames
  if (preview_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    preview_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

  // Set the encode format on the still  port

  format = still_port->format;

  format->encoding = MMAL_ENCODING_OPAQUE;
  format->encoding_variant = MMAL_ENCODING_I420;

  format->es->video.width = state.width;
  format->es->video.height = state.height;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = state.width;
  format->es->video.crop.height = state.height;
  format->es->video.frame_rate.num = 1;
  format->es->video.frame_rate.den = 1;

  status = mmal_port_format_commit(still_port);

  if (status) {
    vcos_log_error("camera still format couldn't be set");
    ROS_ERROR("camera still format couldn't be set");
    goto error;
  }

  video_port->buffer_num = video_port->buffer_num_recommended;
  /* Ensure there are enough buffers to avoid dropping frames */
  if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

  /* Enable component */
  status = mmal_component_enable(camera);

  if (status) {
    vcos_log_error("camera component couldn't be enabled");
    ROS_ERROR("camera component couldn't be enabled");
    goto error;
  }

  raspicamcontrol_set_all_parameters(camera, &state.camera_parameters);

  state.camera_component.reset(camera);

  ROS_DEBUG("Camera component done\n");

  return camera;

error:

  if (camera)
    mmal_component_destroy(camera);
  return 0;
}

/**
 * Create the splitter component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_splitter_component(RASPIVID_STATE& state) {
  MMAL_COMPONENT_T* splitter = 0;
  MMAL_PORT_T* splitter_input = nullptr;
  MMAL_PORT_T *splitter_output_enc = nullptr, *splitter_output_raw = nullptr;
  MMAL_STATUS_T status;
  MMAL_POOL_T* pool;
  MMAL_ES_FORMAT_T* format;

  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_SPLITTER, &splitter);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to create image encoder component");
    ROS_ERROR("Unable to create image encoder component");
    goto error;
  }

  if (!splitter->input_num) {
    status = MMAL_ENOSYS;
    ROS_ERROR("Video splitter doesn't have input ports");
    goto error;
  }

  if (splitter->output_num < 2) {
    status = MMAL_ENOSYS;
    ROS_ERROR("Video splitter doesn't have enough output ports");
    goto error;
  }

  /*** Input Port setup ***/

  splitter_input = splitter->input[0];

  // We want same format on input as camera output
  mmal_format_copy(splitter_input->format, state.camera_component->output[mmal::camera_port::video]->format);

  if (splitter->input[0]->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    splitter->input[0]->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

  // Commit the port changes to the output port
  status = mmal_port_format_commit(splitter_input);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to set format on splitter input port");
    ROS_ERROR("Unable to set format on splitter input port");
    goto error;
  }

  /*** Output to image encoder setup ***/

  splitter_output_enc = splitter->output[0];

  // Copy the format from the splitter input
  mmal_format_copy(splitter_output_enc->format, splitter_input->format);

  status = mmal_port_format_commit(splitter_output_enc);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to set format on splitter output port for image encoder");
    goto error;
  }

  /*** Output for raw ***/

  splitter_output_raw = splitter->output[1];

  // Copy the format from the splitter input
  mmal_format_copy(splitter_output_raw->format, splitter_input->format);

  // Use BGR24 (bgr8 in ROS)
  format = splitter_output_raw->format;
  format->encoding = MMAL_ENCODING_BGR24;
  format->encoding_variant = 0; /* Irrelevant when not in opaque mode */

  status = mmal_port_format_commit(splitter_output_raw);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to set format on splitter output port for raw");
    goto error;
  }

  /*** Setup all other output ports ***/

  // start from 2
  for (unsigned int i = 2; i < splitter->output_num; i++) {
    mmal_format_copy(splitter->output[i]->format, splitter_input->format);

    status = mmal_port_format_commit(splitter->output[i]);

    if (status != MMAL_SUCCESS) {
      vcos_log_error("Unable to set format on splitter output port %d", i);
      goto error;
    }
  }

  /*** Enable component ***/

  status = mmal_component_enable(splitter);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to enable splitter component");
    ROS_ERROR("Unable to enable splitter component");
    goto error;
  }

  /*** Create Pool ***/

  // Create pool of buffer headers for the raw output port to consume
  pool = mmal_port_pool_create(splitter_output_raw, splitter_output_raw->buffer_num, splitter_output_raw->buffer_size);

  if (!pool) {
    vcos_log_error("Failed to create buffer header pool for image encoder output port %s", splitter_output_raw->name);
    ROS_ERROR("Failed to create buffer header pool for image encoder output port %s", splitter_output_raw->name);
  }

  /*** Push to state struct ***/

  state.splitter_pool = mmal::pool_ptr(pool, [splitter](MMAL_POOL_T* ptr) {
    if (splitter->output[1] && splitter->output[1]->is_enabled) {
      mmal_port_disable(splitter->output[1]);
    }
    mmal_port_pool_destroy(splitter->output[1], ptr);
  });

  state.splitter_component.reset(splitter);

  ROS_INFO("splitter component done\n");

  return status;

error:
  if (splitter)
    mmal_component_destroy(splitter);

  return status;
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function
 * successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T* output_port, MMAL_PORT_T* input_port,
                                   mmal::connection_ptr& connection) {
  MMAL_STATUS_T status;

  MMAL_CONNECTION_T* new_connection = nullptr;

  status = mmal_connection_create(&new_connection, output_port, input_port,
                                  MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

  if (status == MMAL_SUCCESS) {
    status = mmal_connection_enable(new_connection);
    if (status != MMAL_SUCCESS)
      mmal_connection_destroy(new_connection);
  }

  connection.reset(new_connection);

  return status;
}

/**
 * init_cam

 */
int init_cam(RASPIVID_STATE& state) {
  MMAL_STATUS_T status;
  MMAL_PORT_T* camera_video_port = nullptr;
  MMAL_PORT_T* splitter_input_port = nullptr;
  MMAL_PORT_T* splitter_output_raw = nullptr;
 
  bcm_host_init();
  // Register our application with the logging system
  vcos_log_register("RaspiVid", VCOS_LOG_CATEGORY);

  // OK, we have a nice set of parameters. Now set up our components
  // We have three components. Camera, splitter and encoder.

  if (!create_camera_component(state)) {
    ROS_ERROR("%s: Failed to create camera component", __func__);
  } else if ((status = create_splitter_component(state)) != MMAL_SUCCESS) {
    ROS_ERROR("%s: Failed to create splitter component", __func__);
    // state.image_encoder_component.reset(nullptr);
    // state.video_encoder_component.reset(nullptr);
    state.camera_component.reset(nullptr);
  } else {
    camera_video_port = state.camera_component->output[mmal::camera_port::video];
    splitter_input_port = state.splitter_component->input[0];
   
    status = connect_ports(camera_video_port, splitter_input_port, state.splitter_connection);
    if (status != MMAL_SUCCESS) {
      ROS_ERROR("%s: Failed to connect camera video port to splitter input", __func__);
      return 1;
    }
    splitter_output_raw = state.splitter_component->output[1];

    PORT_USERDATA* callback_data_raw = new PORT_USERDATA(state);
    callback_data_raw->buffer[0] = std::make_unique<uint8_t[]>(IMG_BUFFER_SIZE);
    callback_data_raw->buffer[1] = std::make_unique<uint8_t[]>(IMG_BUFFER_SIZE);
    // Set up our userdata - this is passed though to the callback where we
    // need the information.
    callback_data_raw->abort = false;
    callback_data_raw->id = 0;
    callback_data_raw->frame = 0;
    splitter_output_raw->userdata = callback_data_raw;
    // Enable the splitter output port and tell it its callback function
    status = mmal_port_enable(splitter_output_raw, splitter_buffer_callback);
    if (status != MMAL_SUCCESS) {
      ROS_ERROR("Failed to setup splitter output");
      return 1;
    }
  state.isInit = true;
  }
  return 0;
}

int start_capture(RASPIVID_STATE& state) {
  if (!(state.isInit))
    ROS_FATAL("Tried to start capture before camera is inited");

  MMAL_PORT_T* camera_video_port = state.camera_component->output[mmal::camera_port::video];
  MMAL_PORT_T* splitter_output_raw = state.splitter_component->output[1];
  ROS_INFO("Starting video capture (%d, %d, %d, %d)\n", state.width, state.height/2, state.quality, state.framerate);

  if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
    return 1;
  }
  int num = mmal_queue_length(state.splitter_pool->queue);
  int q;
  for (q = 0; q < num; q++) {
    MMAL_BUFFER_HEADER_T* buffer = mmal_queue_get(state.splitter_pool->queue);

    if (!buffer) {
      vcos_log_error("Unable to get a required buffer %d from pool queue", q);
      ROS_ERROR("Unable to get a required buffer %d from pool queue", q);
    }

    if (mmal_port_send_buffer(splitter_output_raw, buffer) != MMAL_SUCCESS) {
      vcos_log_error("Unable to send a buffer to splitter output port (%d)", q);
      ROS_ERROR("Unable to send a buffer to splitter output port (%d)", q);
    }
  }
  ROS_INFO("Video capture started\n");
  return 0;
}

int close_cam(RASPIVID_STATE& state) {
  if (state.isInit) {
    state.isInit = false;
    MMAL_COMPONENT_T* camera = state.camera_component.get();
    MMAL_COMPONENT_T* splitter = state.splitter_component.get();

    // Destroy splitter port connection
    state.splitter_connection.reset(nullptr);


    // Destroy splitter component
    if (splitter) {
      // Get rid of any port buffers first
      state.splitter_pool.reset(nullptr);
      // Delete callback structure
      if (splitter->output[1]->userdata) {
        delete splitter->output[1]->userdata;
      }
      state.splitter_component.reset(nullptr);
    }

    // destroy camera component
    if (camera) {
      state.camera_component.reset(nullptr);
    }
    ROS_INFO("Video capture stopped\n");
    return 0;
  } else
    return 1;
}

void reconfigure_callback(raspicam_node::CameraConfig& config, uint32_t level, RASPIVID_STATE& state) {
  ROS_DEBUG("figure Request: contrast %d, sharpness %d, brightness %d, "
            "saturation %d, ISO %d, exposureCompensation %d,"
            " videoStabilisation %d, vFlip %d, hFlip %d,"
            " zoom %.2f, exposure_mode %s, awb_mode %s, shutter_speed %d",
            config.contrast, config.sharpness, config.brightness, config.saturation, config.ISO,
            config.exposure_compensation, config.video_stabilisation, config.vFlip, config.hFlip, config.zoom,
            config.exposure_mode.c_str(), config.awb_mode.c_str(), config.shutter_speed);

  if (!state.camera_component.get()) {
    ROS_WARN("reconfiguring, but camera_component not initialized");
    return;
  }

  if (config.zoom < 1.0) {
    ROS_ERROR("Zoom value %f too small (must be at least 1.0)", config.zoom);
  } else {
    const double size = 1.0 / config.zoom;
    const double offset = (1.0 - size) / 2.0;
    PARAM_FLOAT_RECT_T roi;
    roi.x = roi.y = offset;
    roi.w = roi.h = size;
    raspicamcontrol_set_ROI(state.camera_component.get(), roi);
  }

  raspicamcontrol_set_exposure_mode(state.camera_component.get(), exposure_mode_from_string(config.exposure_mode.c_str()));

  raspicamcontrol_set_awb_mode(state.camera_component.get(), awb_mode_from_string(config.awb_mode.c_str()));

  raspicamcontrol_set_contrast(state.camera_component.get(), config.contrast);
  raspicamcontrol_set_sharpness(state.camera_component.get(), config.sharpness);
  raspicamcontrol_set_brightness(state.camera_component.get(), config.brightness);
  raspicamcontrol_set_saturation(state.camera_component.get(), config.saturation);
  raspicamcontrol_set_ISO(state.camera_component.get(), config.ISO);
  raspicamcontrol_set_exposure_compensation(state.camera_component.get(), config.exposure_compensation);
  raspicamcontrol_set_video_stabilisation(state.camera_component.get(), config.video_stabilisation);
  raspicamcontrol_set_flips(state.camera_component.get(), config.hFlip, config.vFlip);
  raspicamcontrol_set_shutter_speed(state.camera_component.get(), config.shutter_speed);

  ROS_DEBUG("Reconfigure done");
}

int main(int argc, char** argv) {
#ifdef _DEBUG
  DBG_MSG("//DEBUG// - Init rosnode!!!")
#endif  // _DEBUG
  ros::init(argc, argv, "stereo_node");
  ros::NodeHandle nh_params("~");
#ifdef _DEBUG
  DBG_MSG("//DEBUG// - Read parameters!!!")
#endif  // _DEBUG
  bool private_topics;
  nh_params.param<bool>("private_topics", private_topics, true);

  // The node handle used for topics will be private or public depending on the value of the ~private_topics parameter
  ros::NodeHandle nh_topics(private_topics ? std::string("~") : std::string());

  nh_params.param("skip_frames", skip_frames, 0);

  std::string left_camera_info_url, right_camera_info_url;
  std::string left_camera_name, right_camera_name;

  nh_params.param("left_camera_info_url", left_camera_info_url, std::string("package://raspicam_node/camera_info/left_camera.yaml"));
  nh_params.param("right_camera_info_url", right_camera_info_url, std::string("package://raspicam_node/camera_info/right_camera.yaml"));  
  nh_params.param("left_camera_name", left_camera_name, std::string("left_camera"));
  nh_params.param("right_camera_name", right_camera_name, std::string("right_camera"));

  camera_info_manager::CameraInfoManager left_info_man(nh_params, left_camera_name, left_camera_info_url);
    // camera_info_manager::CameraInfoManager right_info_man(nh_params, right_camera_name, right_camera_info_url);

  RASPIVID_STATE state_srv;
#ifdef _DEBUG
  DBG_MSG("//DEBUG// - configure_parameters!!!")
#endif  // _DEBUG
  configure_parameters(state_srv, nh_params);
#ifdef _DEBUG
  DBG_MSG("//DEBUG// - parameters were set!!!")
  DBG_MSG("//DEBUG// - Init camera!!!")
#endif  // _DEBUG
  init_cam(state_srv);
#ifdef _DEBUG
  DBG_MSG("//DEBUG// - Done!!!")
  DBG_MSG("//DEBUG// - Load left camera calibration!!!")
#endif  // _DEBUG
  if(!left_info_man.loadCameraInfo(left_camera_info_url)) {
    ROS_INFO("Calibration file missing. Left camera not calibrated");
  } else {
    left_info = left_info_man.getCameraInfo();
    ROS_INFO("Camera successfully calibrated from config file");
  }
#ifdef _DEBUG
  DBG_MSG("//DEBUG// - Load right camera calibration!!!")
#endif  // _DEBUG
  if (!left_info_man.loadCameraInfo(right_camera_info_url)) {
    ROS_INFO("Calibration file missing. Right camera not calibrated");
  } else {
    right_info = left_info_man.getCameraInfo();
    ROS_INFO("Camera successfully calibrated from config file");
  }

#ifdef _DEBUG
  DBG_MSG("setHardwareID!!!")
#endif  // _DEBUG  

  // diagnostics parameters
  state_srv.updater.setHardwareID("raspicam");
  double desired_freq = state_srv.framerate;
  double min_freq = desired_freq * 0.95;
  double max_freq = desired_freq * 1.05;
#ifdef _DEBUG
  DBG_MSG("//DEBUG// - Advertizing!!!")
#endif  // _DEBUG
  auto left_image_pub = nh_topics.advertise<sensor_msgs::Image>("left_image", 2);
  left_image.pub.reset(new DiagnosedPublisher<sensor_msgs::Image>(
      left_image_pub, state_srv.updater, FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10), TimeStampStatusParam(0, 0.2)));  // if (state_srv.enable_imv_pub) {
  auto right_image_pub = nh_topics.advertise<sensor_msgs::Image>("right_image", 2);
  right_image.pub.reset(new DiagnosedPublisher<sensor_msgs::Image>(
      right_image_pub, state_srv.updater, FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10), TimeStampStatusParam(0, 0.2)));
  if(left_image.pub == nullptr || right_image.pub == nullptr)
  {
    ROS_INFO("Publisher is corrupted");
    return 0;
  }
  left_camera_info_pub = nh_topics.advertise<sensor_msgs::CameraInfo>("left_camera_info", 2);
  right_camera_info_pub = nh_topics.advertise<sensor_msgs::CameraInfo>("right_camera_info", 2);

  dynamic_reconfigure::Server<raspicam_node::CameraConfig> server;
  dynamic_reconfigure::Server<raspicam_node::CameraConfig>::CallbackType f;
  f = boost::bind(&reconfigure_callback, _1, _2, boost::ref(state_srv));
#ifdef _DEBUG
  DBG_MSG("//DEBUG// - Setting up Dynamic reconfigure!!!")
#endif  // _DEBUG
  server.setCallback(f);
#ifdef _DEBUG
  DBG_MSG("//DEBUG// - start_capture!!!")
#endif  // _DEBUG
  start_capture(state_srv);
  ros::spin();
  close_cam(state_srv);
  ros::shutdown();
}

#endif  // __arm__
