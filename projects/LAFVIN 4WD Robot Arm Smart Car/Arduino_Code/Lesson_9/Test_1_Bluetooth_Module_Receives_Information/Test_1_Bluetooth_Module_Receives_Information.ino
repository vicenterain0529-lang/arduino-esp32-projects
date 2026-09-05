String BLE_val;

void setup(){
  Serial.begin(9600);
  BLE_val = "";
}

void loop(){
  while (Serial.available() > 0) {
    BLE_val = BLE_val + ((char)(Serial.read()));
    delay(2);
  }
  if (0 < String(BLE_val).length() && 2 >= String(BLE_val).length()) {
    Serial.println(BLE_val);
    BLE_val = "";

  }

}
