
#include "HX711.h"
#include <ros.h>
#include <std_msgs/Float64.h>

#define calibration_factor 7000 //This value is obtained using the SparkFun_HX711_Calibration sketch

#define LOADCELL_DOUT_PIN  3
#define LOADCELL_SCK_PIN  2

ros::NodeHandle node_handle;

std_msgs::Float64 force_msg;

ros::Publisher force_publisher("/force_reading", &force_msg);

HX711 scale;

void setup() {
  node_handle.initNode();
  node_handle.advertise(force_publisher);
  
  Serial.begin(9600);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0

  Serial.println("Readings:");
}

void loop() {
  Serial.print("Reading: ");
  double reading = scale.get_units();
  force_msg.data = reading;
  Serial.print(reading, 3);
  Serial.print(" kgs");
  Serial.println();

  force_publisher.publish( &force_msg );
  node_handle.spinOnce();

  delay(150);

  
}
