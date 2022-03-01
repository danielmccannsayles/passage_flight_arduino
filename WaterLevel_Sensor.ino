int topOpticalSensorPin = 0;                                 // the cell and 4.7K pulldown are connected to analog pin A0
int WaterHeight;                                // variable to hold the optical float sensor


// MAIN PROGRAM SETUP

 
void setup() {
  Serial.begin(9600);                                                                 // Initialise serial communications
}



// MAIN PROGRAM LOOP
void loop() {
  WaterLevel();
    delay(1000);
}


void WaterLevel() {
  WaterHeight = analogRead(topOpticalSensorPin);     // take a reading from the optical sensor pin
 
  Serial.print("Water Height Reading = ");                     // Print string to console
  Serial.print(WaterHeight);                                      // the analog reading of the optical sensor
 
  if (WaterHeight < 100) {
    Serial.println(" - TOUCHING WATER - YES");} 
    else if (WaterHeight < 400) {
    Serial.println(" - TOUCHING WATER - NO");} 
    else {
    Serial.println(" - TOUCHING WATER - NO");
    }

}
