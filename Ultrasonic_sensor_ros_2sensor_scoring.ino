#include <ros.h>
#include <sensor_msgs/Range.h>
#include <ros/time.h>
#include <new_message/Ranges.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
ros::NodeHandle nh;
new_message::Ranges values_msg;
std_msgs::Int32 data_wet;
void messageCb( const std_msgs::Bool& toggle_msg);
ros::Publisher pub_range("range_data",&values_msg);
ros::Publisher pub_wet("Wet_data", &data_wet);
ros::Subscriber<std_msgs::Bool> sub("toggle_pump", &messageCb );
char frameid[] = "/ultrasound";
const int trigPin1=6;
const int echoPin1=7;
const int trigPin2=8;
const int echoPin2=9;
const int array_len=9;
float maxRange=200;
float maxDriff=10;
float maxVar=10;
float final_range=0;
float array_range1[array_len];
float array_range2[array_len];
float var1; 
float var2;
int score1;
int score2;

long duration;
float distance1;
float distance2;
void messageCb( const std_msgs::Bool& toggle_msg){
  bool trig = toggle_msg.data;
  if(trig)
  digitalWrite(13, HIGH);   // blink the led
  else digitalWrite(13, LOW); 
}
void swap(float& a, float& b)
{
   float temp;
   temp  = a;
   a=b;
   b=temp;
}
float median_filter(float* arr)
{
  float arr_sorted[array_len];
  for (int i=0; i<array_len; i++)
  {
    arr_sorted[i]=arr[i];
  }
  
  for (int i=0; i<array_len; i++)
  {
    for (int j=i; j<array_len; j++)
    {
      if(arr_sorted[i]>arr_sorted[j])
      {
        swap(arr_sorted[i],arr_sorted[j]);
      }
    }
  }
  return arr_sorted[array_len/2+1];
}

void input_value(float* arr, float new_value)
{
  for (int i=0; i<(array_len-1); i++)
  {
    swap(arr[i],arr[i+1]);
  }
  arr[array_len-1]=new_value;
}

float variance_calculate(float* arr)
{
  int sum=0;
  for (int i=0; i<array_len; i++)
  {
    sum+=arr[i];
  }
  float avg=sum/array_len;
  sum = 0;
  for (int i=0; i<array_len; i++)
  {
    sum+=(arr[i]-avg)*(arr[i]-avg);
  }
  return sum;
}

int scoring_sensor(float current_value, float variance)
{
  int score=0;
  if (current_value<maxRange) score+=4;
  if ((current_value-final_range)< maxDriff) score+=2;
  if (variance<maxVar) score+=1;
  return score;
}
float measure_distance(int sensor_number)
{
  float distance;
  int trigPin=trigPin1;
  int echoPin=echoPin1;
  if (sensor_number==2)
  {
    trigPin=trigPin2;
    echoPin=echoPin2;
  }

  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  duration=pulseIn(echoPin,HIGH);
  distance=duration*0.034/2;
  duration=0;
  return distance;
}
void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pub_range);
  pinMode(trigPin1,OUTPUT);
  pinMode(echoPin1,INPUT);
  pinMode(trigPin2,OUTPUT);
  pinMode(echoPin2,INPUT);
  pinMode(13, OUTPUT);
  nh.subscribe(sub);


  for (int i=0; i<array_len; i++)
  {
    array_range1[i]=measure_distance(1);
    delay(100);
    array_range2[i]=measure_distance(2);
    delay(100);
  }
  Serial.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float raw_distance1=measure_distance(1);
  input_value(array_range1, raw_distance1);
  distance1=median_filter(array_range1);
  var1=variance_calculate(array_range1);
  score1=scoring_sensor(distance1,var1);
  delay(100);
  float raw_distance2=measure_distance(2);
  input_value(array_range2, raw_distance2);
  distance2=median_filter(array_range2);
  var2=variance_calculate(array_range2);
  score2=scoring_sensor(distance2,var2);

  if (score1==score2) final_range=(distance1+distance2)/2;
  else if (score1>score2) final_range=distance1;
  else final_range=distance2;
  /*Serial.print("Raw Distance 1: ");
  Serial.print(raw_distance1);
  Serial.print("Distance 1: ");
  Serial.print(distance1);
  Serial.print("\n");*/

  /*Serial.print(raw_distance1);
  Serial.print(" ");
  Serial.print(raw_distance2);
  Serial.print(" ");*/

  /*Serial.print(distance1);
  Serial.print(" ");
  Serial.print(distance2);
  Serial.print(" ");
  Serial.println(final_range);*/

  /*Serial.print(score1);
  Serial.print(" ");
  Serial.println(score2);*/
  
 
  //pulishing data
  values_msg.Range1=analogRead(A1);
  values_msg.Range2=distance2;
  values_msg.fusedRange=final_range;
 // data_wet.data = analogRead(A1);
  pub_range.publish(&values_msg);
  ///pub_wet.publish(&data_wet);
  
  nh.spinOnce();
  delay(100);
}
