# firstproject
learning about git

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

TinyGPS gps;

char apn[]  = "";
char user[] = "";
char pass[] = "";


void setup()
{
  Serial.begin(9600);
//  Serial1.begin(9600);
  Serial2.begin(9600);
  lcd.begin(20, 4);
  Wire.begin();
  accelgyro.initialize();
  pinMode(rest_button, INPUT_PULLUP);
  pinMode(relay, OUTPUT);
  pinMode(manual_switch, INPUT_PULLUP);
  lati[0] = 13.932734; longi[0] = 75.582439;  //BH ROAD
  lati[1] = 13.952596; longi[1] = 75.560463; //VINOB NAGARA
  lati[2] = 13.934674; longi[2] = 75.550870; //VISL Criket stadium 
  lati[3] = 12.942853; longi[3] = 77.574481; //kadada katte
  lcd.setCursor(0, 0);
  lcd.print("Smrt Vehicl Accident");
  lcd.setCursor(0, 1);
  lcd.print("  Detection System  ");
  lcd.setCursor(0, 2)
  lcd.print("From: IS Department.");
  lcd.setCursor(0, 3);
  lcd.print("  PESITM Shivamogga ");
  int h = 1;
  String ok = "";
  delay(5000);
  lcd.clear();

  SerialAT.begin(115200);
  delay(3000);
  Serial.println("Initializing modem...");
  modem.restart();
  modem.simUnlock("1234");
  Blynk.begin(auth, modem, apn, user, pass);
}


void loop()
{
  z=1; i=1;
  Blynk.run();
  Blynk.virtualWrite(V0, analogRead(A0));
  sensor_data();
  gps_data();
  IMU6050();
  MANUAL_sms();
}

void sensor_data()
{
  int sensor_value = analogRead(A0);
  Blynk.virtualWrite(V0, sensor_value);
  if (sensor_value >= 310)
  {
    digitalWrite(relay, 1);
    lcd.setCursor(0, 0);
    lcd.print("Alc:");
    lcd.print(sensor_value);
    lcd.print("  ");
    lcd.setCursor(10, 0);
    lcd.print("Engn:");
    lcd.print(" OFF");
    lcd.setCursor(0, 1);
    lcd.print("   Driver Drunked   ");
    Blynk.virtualWrite(V1, LOW);
  }
  else if (sensor_value <= 300)
  {
    digitalWrite(relay, 0);
    lcd.setCursor(0, 0);
    lcd.print("Alc:");
    lcd.print(sensor_value);
    lcd.print("  ");
    lcd.setCursor(10, 0);
    lcd.print("Engn:");
    lcd.print(" ON ");
    lcd.setCursor(0, 1);
    lcd.print("Driver is All Right ");
    Blynk.virtualWrite(V1, HIGH);
  }
}

void gps_data()
{
  while (Serial2.available() > 0)
  {
    if (gps.encode(Serial2.read()))
    {
      gps.f_get_position(&latitude, &longitude); //      Serial.println(latitude, 6);
      lcd.setCursor(0, 2);  lcd.print("Lat:");  lcd.print(latitude, 6);
      lcd.setCursor(0, 3);  lcd.print("Lon:"); lcd.print(longitude, 6);
      if ( latitude > 0 && longitude > 0 )
      {
        previous_lati = latitude;
        previous_longi = longitude;
        Blynk.virtualWrite(V2,1,previous_lati,previous_longi,"Car");
      }
      Serial.print("Latitude: ");    Serial.print(latitude, 6);
      Serial.print("  Longitude: "); Serial.println(longitude, 6);
    }
  }
}

void IMU6050()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  if (gy > 1000 )
  {
    Serial.println("your vehicle met with accident");
    Blynk.notify("Your Vehicle Met with Accident");
    delay (1000);
    if (i == 1)
    { 
      for (int w = 0; w <= 10; w++)
      {
        timer();
        i = 0;
      }
    }

    if (seconds >= 10)
    {
      count = 0;
      seconds = 0;
      i = 1;
      Serial.println("SECONDS10");
      Distance_measure();
    }
  }
}

void timer()
{
  for (unsigned int isr = 0; isr <= 1000; isr++)
  {
    delay(1);
    count++;
    if (count == 1000)
    {
        lcd.setCursor(0, 1);
        lcd.print("PRESS SWITCH:");
        lcd.print(" ");
        lcd.print(seconds);
        lcd.print(" Sec");
      seconds++;
      Serial.println("timer");
      count = 0;
    }
    if (digitalRead(rest_button) == 0)
    {
      count = 0;
      seconds = 0;
      i = 1;
      lcd.setCursor(0, 1);
      lcd.print("MESSAGE TERMINATED..");
      delay(1000);
    }
  }
}

void ascending_order()
{
  int a, s, d, f = 3;
  for (a = 0; a <= f; a++)
  {
    for (s = a + 1; s <= f; s++)
    {
      if (distance_arra[a] > distance_arra[s])
      {
        d = distance_arra[a];
        distance_arra[a] = distance_arra[s];
        distance_arra[s] = d;
      }
    }
    for (int p = 0; p <= 3; p++)
    {
      distance[p] = distance_arra[p];
      Serial.println(distance_arra[p]);
    }
  }
}

void Distance_measure()
{
  for (int k = 0; k <= 3; k++)
  {
    distance_arra[k] = gps.distance_between (lati[k], longi[k], latitude, longitude) / 1000;
    Serial.println(distance_arra[k]);
  }
  for (int o = 0; o <= 3; o++)
  {
    pre[o] = distance_arra[o];
  }
  ascending_order();
  for (int v = 0; v < 1; v++)
  {
    for (int l = 0; l <= 3; l++)
    {
      if (pre[l] == distance[v])
      {
        if (z == 1)
        {
          lcd.setCursor(0, 1);
          lcd.print("MESSAGE SENDING.....");
          e = 0;
          sendsms(gardian ); // gardian
          e++;
          Serial.print(l);
          sendsms(police_no[l]);
          Serial.print(l);
          e++;
          sendsms(hospital_no[l]);
          Serial.print(l);
          z = 0;
        }
      }
    }
  }
}



bool sendsms(char* number)
{
  Serial.println(e);
  Serial1.print("AT+CMGF=1\r"); //set sms to text mode
  _buffer = _readSerial();
  Serial.println(_buffer);
  Serial1.print("AT+CMGS=\"");  Serial.println(number);
  Serial1.print (number);
  Serial1.print("\"\r");
  _buffer = _readSerial(); Serial.println(_buffer);
  if (manual == 1)
  {
    if (e == 0)
    {
      Serial1.print ("Your vehicle met with accident.");
      Serial1.print(" the live location is here. http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=");
    }
    else if (e == 1)
    {
      Serial1.print ("accident happened.");
      Serial1.print ("km/h");
      Serial1.print(" the live location is here. http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=");
    }
    else if (e == 2)
    {
      Serial1.print ("This is a emergency message. Accident happened need medical help.the live location is here. http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=");
    }
  }
  else
  {
    if (e == 0)
    {
      Serial1.print ("SOS Emergency Help Needed");
      Serial1.print(" the live location is here. http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=");
    }
    else if (e == 1)
    {
      Serial1.print ("Emergency Help Needed");
      Serial1.print ("km/h");
      Serial1.print(" the live location is here. http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=");
    }
    else if (e == 2)
    {
      Serial1.print ("Emergency Help Needed.the live location is here. http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=");
    }
    manual = 1;
  }
  Serial1.print(latitude, 6);
  Serial1.print(",");
  Serial1.print(longitude, 6);
  Serial1.print ("\r");
  _buffer = _readSerial();  Serial.println(_buffer);
  Serial1.print((char)26);
  _buffer = _readSerial();  Serial.println(_buffer);
  if (((_buffer.indexOf("CMGS") ) != -1 ) )
  {
    return true;
  }
  else
  {
    return false;
  }
  Serial.println(e); Serial.println("e");
  if (e >= 3)e = 0;
}

void MANUAL_sms()
{
  if ( digitalRead(manual_switch) == 0 )
  {
    Serial.println("SOS Need Emergency Help");
    Blynk.notify("SOS Need Emergency Help");
    manual = 0;
    z=1;
        if (i == 1)
    {
      for (int w = 0; w <= 10; w++)
      {
        timer();
        i = 0;
      }
    }

    if (seconds >= 10)
    {
      count = 0;
      seconds = 0;
      i = 1;
      Serial.println("SECONDS10");
      Distance_measure();
    }
  }
}
