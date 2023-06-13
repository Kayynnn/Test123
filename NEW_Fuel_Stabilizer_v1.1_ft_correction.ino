#include <SimpleKalmanFilter.h>

int validation_threshold = 120; // banyak validasi sampling
int ft_detection = 0;
int count_false_validation = 0;
int count_bad_data = 0;
int count_sampling_amount = 0; /// nih yg ini
int ft_validation = 0;
int ft_benchmark_id[150];
int start_detection[150];
int break_validation[150];
int valid_id;
float sum_sampling;
float sum_bad_data;
int id_check_point;
bool first_detect = true;


int calibration_mode = D5; //for activate/deactivate calibration mode, LOW = active, HIGH = deactive || ganti jadi D5
float SensorData, KalmanFilterData, measured_value;
int delay_on = 180;//delay ketika ignition nyala (detik)
int delay_sampling = 1; // delay sebelum pengambilan sampling dalam detik
const int delay_mean = 300; // delay sebelum meratatakan output dalam detik

float threshold_ft = 0.2; //threshold filling or theft dalam Volt


float measured_samplings[delay_mean] = {

  /*255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255*/
};

float input_data = A0; //Pin input dari fuel tank
int fuel_output = D6; //Pin output fuel ke rangkaian LPF
int ignition = D2; //Trigger untuk deteksi ignition
int start = 0;

int led_pin = D1; //led indicator
int led_state = LOW;
unsigned long led_prev = 0;
const long led_interval = 1000;

float sum = 0;
int mean = 0;

unsigned long count = 0;
unsigned long tod_b = millis(); //untuk timer on delay pembacaan fuel
int first_on = 0;
int first_ignition = 1;
unsigned long b = 0;
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
void setup()
{
  //convert our input value(voltage) to pwm
  threshold_ft = (threshold_ft / 3.3) * 255;

  Serial.begin(9600);
  Serial.println(abs(measured_samplings[12] - measured_samplings[13]));
  Serial.println(abs(measured_samplings[13] - measured_samplings[12]));
  Serial.println(threshold_ft);
  //delay(3000);
  pinMode(fuel_output, OUTPUT);
  pinMode(ignition, INPUT);
  pinMode(calibration_mode, INPUT);
//  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(led_pin, OUTPUT);
  digitalWrite(calibration_mode, LOW);
}
void loop()
{
  unsigned long led_current = millis();
  if (digitalRead(ignition) == HIGH && digitalRead(calibration_mode) == HIGH) {

    // led start blinking
    if (start < delay_on) {
      if (led_current - led_prev >= led_interval / 10) {
        led_prev = led_current;
        if (led_state == LOW) {
          led_state = HIGH;
        }
        else {
          led_state = LOW;
        }
        digitalWrite(led_pin, led_state);
      }
    }
    if (first_ignition == 1) {
      //wait 3mins
      tod_b = millis() / 1000;
      first_ignition = 0;

      //      digitalWrite(led_pin, LOW);
    }
    unsigned long tod_a = millis() / 1000;
    if (tod_a - tod_b >= delay_on) {
      //      digitalWrite(led_pin, HIGH);
      unsigned long a = millis() / 1000;
      //            Serial.print("   ||   a : ");
      //            Serial.print(a);
      SensorData = analogRead(input_data);
      SensorData = map(SensorData, 0, 1023, 0, 255);
      Serial.print("ADC Lgsg : ");
      Serial.print(SensorData);
      Serial.print("; ");
      Serial.print("  ||  Fuel Sensor : ");
      KalmanFilterData = simpleKalmanFilter.updateEstimate(SensorData);
      Serial.print(KalmanFilterData);
      if (first_on != 1) {
        b = millis() / 1000;
      }
      sum += KalmanFilterData;
      count++;
      //      Serial.print(" || a-b : ");
      //      Serial.print(a - b);
      if (a - b >= delay_sampling) {
        b = a;
        measured_samplings[count_sampling_amount] = sum / count;
        Serial.print("   ||   SUM  : ");
        Serial.print(sum);
        Serial.print("  ||  Count : ");
        Serial.print(count);
        sum = 0;
        count = 0;

        Serial.print("\n");
        Serial.print("Measured Sampling " + String(count_sampling_amount + 1) + " : ");
        Serial.println(measured_samplings[count_sampling_amount]);

        if (count_sampling_amount > 0 && abs(measured_samplings[count_sampling_amount] - measured_samplings[count_sampling_amount - 1]) > threshold_ft) {

          if (abs(id_check_point - count_sampling_amount) >= 5 or first_detect) {
            ft_benchmark_id[ft_detection] = count_sampling_amount - 1;
            ft_detection += 1;
            first_detect = false;
          }
          id_check_point = count_sampling_amount;
          //delay(5000);
        }

        count_sampling_amount += 1;


      }
      digitalWrite(led_pin, HIGH);
    }



    if (count_sampling_amount == delay_mean) {
      Serial.print("Ft Detection : ");
      Serial.println(ft_detection);
      //delay(2000);

      if (ft_detection > 0) {
        Serial.println("KEDETECT WOE");
        //delay(2000);
        for (int i = 0; i < ft_detection; i++) {
          Serial.print("|| ft_detection : ");
          Serial.print(ft_detection);
          Serial.print(" ||");

          //delay(2000);
          if (i % 2 == 0) {
            for (int n = 0; n < validation_threshold; n++) {
              Serial.print("n : ");
              Serial.println(n);
              //delay(500);
              //            Serial.print("Data A  : ");
              //
              //            Serial.println(measured_samplings[ft_benchmark_id[i] + 2 + n]);
              //            Serial.print("Data pembanding ");
              //            Serial.print(ft_benchmark_id[i]);
              //            Serial.print(" : ");
              //            Serial.println(measured_samplings[ft_benchmark_id[i]]);
              //            Serial.println(abs(measured_samplings[ft_benchmark_id[i] + 2 + n] - measured_samplings[ft_benchmark_id[i]]));

              //delay(1500);
              if (abs(measured_samplings[ft_benchmark_id[i] + 2 + n] - measured_samplings[ft_benchmark_id[i]]) >= threshold_ft) {
                if (n == validation_threshold - 1) {

                  ft_validation += 1;
                  valid_id = ft_benchmark_id[i] + 1;
                }


              } else {
                start_detection[count_false_validation] =  ft_benchmark_id[i] + 1;
                break_validation[count_false_validation] = ft_benchmark_id[i] + 2 + n;

                count_false_validation += 1;
                n = 99 + validation_threshold;
              }
            }
          }
        }

        Serial.print("|| Count false validation : ");
        Serial.print(count_false_validation);
        Serial.print(" ||");
        Serial.print("|| Validation : ");
        Serial.print(ft_validation);
        Serial.print(" ||");

        if (ft_validation > 0) {
          int ft_voltage = measured_samplings[delay_mean];
          Serial.print(" valid id : ");
          Serial.println(valid_id);
          //delay(2000);
          if (valid_id <= delay_mean - validation_threshold) {
            mean = ft_voltage;

          } /*else {
            sum_sampling = 0;
            for (int i = 0; i < valid_id; i++) {
              sum_sampling += measured_samplings[i];
            }*/
          //mean = sum_sampling / valid_id;

          ft_validation = 0;
        } else {
          Serial.print("|| Count false validation : ");
          Serial.print(count_false_validation);
          Serial.print(" ||");
          sum_sampling = 0;
          sum_bad_data = 0;
          for (int i = 0; i < delay_mean ; i++) {
            sum_sampling += measured_samplings[i];
          }
          for (int i = 0; i < count_false_validation; i++) {
            for (int n = start_detection[i]; n < break_validation[i]; n++) {
              sum_bad_data += measured_samplings[n];
              count_bad_data += 1;
            }
          }


          sum_sampling -= sum_bad_data;
          mean = sum_sampling / (delay_mean - count_bad_data);
                 //          Serial.print("|| Sum bad data : ");
                 //          Serial.print(sum_bad_data);
                 //          Serial.print("|| Sum Sampling : ");
                 //          Serial.print(sum_sampling);
                 //          Serial.print("|| Count Bad Data : ");
                 //          Serial.print(count_bad_data);
                 //delay(2500);
                 count_bad_data = 0;
          count_false_validation = 0;
        }
        ft_detection = 0;
      } else {
        sum_sampling = 0;
        for (int i = 0; i < delay_mean; i++) {
          sum_sampling += measured_samplings[i];
        }
        mean = sum_sampling / delay_mean;
      }
      count_sampling_amount = 0;
      first_on = 1;
      id_check_point = count_sampling_amount;
      first_detect = true;
    }




    start = int(tod_a - tod_b);
    Serial.print(" || ");
    Serial.print(start);
    analogWrite(fuel_output, mean);
    Serial.print("  ||  Mean : ");
    Serial.println(mean);
//    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (first_on == 1) {
    tod_b = millis() / 1000;
  }
  else {
    sum = 0;
    count = 0;
    digitalWrite(led_pin, LOW);
//    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (digitalRead(calibration_mode) == LOW) {
    digitalWrite(led_pin, LOW);
//    digitalWrite(LED_BUILTIN, LOW);
    SensorData = analogRead(input_data);
    SensorData = map(SensorData, 0, 1023, 0, 255);
    Serial.print("Calibration Mode  ||  Fuel Sensor : ");
    KalmanFilterData = simpleKalmanFilter.updateEstimate(SensorData);
    Serial.println(KalmanFilterData);
    analogWrite(fuel_output, KalmanFilterData);
  }

  if (digitalRead(ignition) == LOW) {
    digitalWrite(led_pin, LOW);
//    digitalWrite(LED_BUILTIN, HIGH);
    first_ignition = 1;
    analogWrite(fuel_output, mean);
    Serial.print("  ||  ");
    Serial.println(mean);
  }
  //Serial.println(digitalRead(ignition));
}
