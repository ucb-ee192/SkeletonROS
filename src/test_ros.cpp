//
// Referenced Louis Whitcomb's edumip_balance_ros.cpp:
// https://git.lcsr.jhu.edu/lwhitco1/edumip_balance_ros

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "test_ros/LedLag.h"
#include <stdio.h>
#include <robotcontrol.h>
#include <rc_usefulincludes.h>

float start_t;
unsigned long long start_ts;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	if (rc_get_state()==EXITING){
		return;
	}

	unsigned int threshold = 10; // Subtract this from all pixels

	unsigned int h = msg->height; 	// rows
	unsigned int w = msg->width; 	// columns
	unsigned int pix_bytes = msg->step/msg->width; // bytes per pixel
	unsigned int i, j, k; 	// for iteration
	float t = msg->header.stamp.toSec() - start_t; // elapsed time

	unsigned int pix; // Current pixel value
	unsigned int mean; // Mean brightness of all pixels
	unsigned int center; // Centroid of image brightness
	unsigned long sum = 0; // Sum over all pixel brightness
	unsigned long weighted_sum = 0; // For computing centroid

	unsigned int g = 0;
	unsigned int r = 0;

	// Loop over image pixels
	for (i=0; i<h; i++) // rows
	{
		for (j=0; j<w; j++) // columns
		{
			for (k=0; k<pix_bytes; k++) // pixel colors
			{
				// Read pixel
				pix = msg->data[i*w*pix_bytes+j*pix_bytes+k];
				if (pix >= threshold)
				{
					pix -= threshold;
				} else {
					pix = 0;
				}

				// Compute image mean brightness and centroid
				sum += pix;
				weighted_sum += j*pix;

				if (k == 0)
				{
					r += pix;
				} else if (k == 1)
				{
					g += pix;
				}
			}
		}
	}
	mean = sum/(h*w*pix_bytes);
	if (sum > 0)
	{
		center = weighted_sum/sum;
	} else {
		center = w/2;
	}

	g = g/(h*w);
	r = r/(h*w);
	
	// Set steering servo output
	unsigned int steer = (1500 + (center-w/2)*100)*1000;
	steer = steer > 1900000? 1900000 :
		steer < 1100000? 1100000 :
		steer;
	rc_pwm_set_duty_ns(0, 'B', steer);

	// Set drive motor PWM
	rc_motor_set(1, (float)mean/256.0);
	
	// Read wheel encoder
	int enc_pos = rc_encoder_eqep_read(1);

	// Print image stats
	//printf("Time: %.2f, Center: %d, Bright: %d Encoder: %d\n",
	//	t, center, mean, enc_pos);
	//printf("G: %d, R: %d T: %lld\n", g, r,
	//		rc_nanos_since_boot()/1000000000);

	if (g > r+3) {
		unsigned long long lag=(rc_nanos_since_boot()-start_ts)/1000;
		printf("TIME: %lld\n", lag);
		rc_gpio_set_value(3, 2, 0);
	}
}

bool handle_service(test_ros::LedLag::Request &req,
		    test_ros::LedLag::Response &res) {
	res.out = 0;
	start_ts = rc_nanos_since_boot();
	rc_gpio_set_value(3, 2, 1);
	unsigned long long lag=(rc_nanos_since_boot()-start_ts)/1000;
	ROS_INFO("Turning on LED, %lld (us)", lag);
	return true;
}

void ros_rc_cleanup(void)
{
	// Run Robot Control Library cleanup functions on exit
	rc_motor_cleanup();
	rc_pwm_cleanup(0);
	rc_encoder_eqep_cleanup();
	rc_gpio_cleanup(3, 2);
}

void ros_compatible_shutdown_signal_handler(int signo)
{
	// Catch shutdown signals and run cleanup functions
	if (signo == SIGINT)
	{
		rc_set_state(EXITING);
		ros_rc_cleanup();
		ROS_INFO("Received SIGINT.");
		ros::shutdown();
	}
	else if	(signo == SIGTERM)
	{
		rc_set_state(EXITING);
		ros_rc_cleanup();
		ROS_INFO("Received SIGTERM.");
		ros::shutdown();
	}
}

int main(int argc, char **argv)
{
	// Initialize ROS functionality
	ROS_INFO("STARTED!");
	ros::init(argc, argv, "test_ros");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1,
			image_callback);
	ros::ServiceServer service = n.advertiseService("led_lag",
			handle_service);

	// Initialize signal handler
	signal(SIGINT, ros_compatible_shutdown_signal_handler);
	signal(SIGTERM, ros_compatible_shutdown_signal_handler);
	
	// Get the start time
	start_t = ros::Time::now().toSec();

	// Initialize GPS connector PWM
	rc_pinmux_set(GPS_HEADER_PIN_4,PINMUX_PWM);
	rc_pwm_init(0, 50);
	rc_pwm_set_duty_ns(0, 'B', 1500000);

	// Initialize H-bridge PWM
	rc_motor_init_freq(25000);

	// Initialize GPIO pin
	rc_gpio_init(3, 2, GPIOHANDLE_REQUEST_OUTPUT);
	
	// Intialize quadrature encoder reader
	rc_encoder_eqep_init();

	ros::spin();
	
	ROS_INFO("EXITING!");
	return 0;
}
