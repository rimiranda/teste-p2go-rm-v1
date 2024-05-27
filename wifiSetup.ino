void wifiSetup() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the Wi-Fi network");
  //   connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  checkConnection();
  //  Publish and subscribe
  //  client.subscribe(sub_topic);
}

void checkConnection() {
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      client.subscribe(TOPIC_SUBSCRIBE_LED);
      Serial.println("Broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
    //    client.subscribe(sub_topic);
  }
}

void callback(char *topic, byte * payload, unsigned int length) {
  
  String msg;

  Serial.print("Mensagem recebida no topico: ");
  Serial.println(topic);
  Serial.print("Mensagem:");
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println();

 // Obtem a string do payload recebido
  for (int i = 0; i < length; i++) {
    char c = (char)payload[i];
    msg += c;
  }

  /* Muda a cor do led*/
  if (msg.equals("tcl")) {
    flagCorLed++; 
    if (flagCorLed > 4){flagCorLed = 1;}
    Serial.print("Cor do LED alterada por comando MQTT");
  }
  /* Muda o estado do led*/
  if (msg.equals("tsl")) {
    flagStateLed++;
    if (flagStateLed > 3){flagStateLed = 0;}
    Serial.print("Estado do LED alterado por comando MQTT");
  }



}