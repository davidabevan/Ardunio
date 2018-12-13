/*
 Web Client to consume Weather Underground web service
 
 This sketch connects to a website (http://api.wunderground.com)
 using an Arduino Ethernet shield and get data from site.
 
 Circuit:
 * Arduino MEGA 2560 R3 Board
 * Ethernet shield attached to pins 10, 11, 12, 13
 
 created 07 Jan 2014
 by Afonso C. Turcato
 
 */

#include <SPI.h>
#include <Ethernet.h>

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(23,216,168,120);  // numeric IP for Weather Underground  (no DNS)
const char server[] = "api.wunderground.com";    // name address for Weather Underground (using DNS)

const String myKey = "xxxxxxxxxxxxxxxx";  //See: http://www.wunderground.com/weather/api/d/docs (change here with your KEY)
const String myFeatures = "conditions";   //See: http://www.wunderground.com/weather/api/d/docs?d=data/index&MR=1
const String myCountry = "Brazil";        //See: http://www.wunderground.com/weather/api/d/docs?d=resources/country-to-iso-matching
const String myCity = "Sertaozinho";      //See: http://www.wunderground.com/weather/api/d/docs?d=data/index&MR=1

// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192,168,1,177);

// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 80 is default for HTTP):
EthernetClient client;

//Response from Server
String responseString;

boolean startCapture;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip);
  }
  
  // give the Ethernet shield a half-second to initialize:
  delay(500);
  Serial.print("My IP Address is: ");
  Serial.println(Ethernet.localIP());
  Serial.println("Connecting...");
  
  // if you get a connection, report back via serial:
  if (client.connect(server, 80))
  {
    Serial.println("Connected!");
    
    String html_cmd1 = "GET /api/" + myKey + "/" + myFeatures + "/q/" + myCountry + "/" + myCity + ".json HTTP/1.1";
    String html_cmd2 = "Host: " + (String)server;
    String html_cmd3 = "Connection: close";
    
    //Uncomment this is necessary
    //Serial.println("Sending commands:");
    //Serial.println(" " + html_cmd1);
    //Serial.println(" " + html_cmd2);
    //Serial.println(" " + html_cmd3);
    //Serial.println();
    
    // Make a HTTP request:
    client.println(html_cmd1);
    client.println(html_cmd2);
    client.println(html_cmd3);
    client.println();
    
    responseString = "";
    startCapture = false;   
  } 
  else
  {
    // if you didn't get a connection to the server:
    Serial.println("Connection failed!");
  }
}

void loop()
{
  // if there are incoming bytes available 
  // from the server, read them and buffer:
  if (client.available())
  {
    char c = client.read();
    if(c == '{')
      startCapture=true;
    
    if(startCapture)
      responseString += c;
  }
  
  // if the server's disconnected, stop the client:
  if (!client.connected()) {    
    Serial.println("Received " + (String)responseString.length() + " bytes");
    Serial.println("Disconnecting.");
    client.stop();
    client.flush();
    
    Serial.println();
    
    //Now, some examples of how to use it!
    Serial.print("Weather: ");
    Serial.println(getValuesFromKey(responseString, "weather"));
    Serial.println();
    
    Serial.print("Current Temperature: ");
    Serial.print(getValuesFromKey(responseString, "temp_c"));
    Serial.println(" oC\n");
    
    Serial.print("Relativy Humidity: ");
    Serial.println(getValuesFromKey(responseString, "relative_humidity"));
    Serial.println();
    
    Serial.print("Wind: ");
    Serial.println(getValuesFromKey(responseString, "wind_string"));
    Serial.println();
    
    Serial.print("Feels like: ");
    Serial.print(getValuesFromKey(responseString, "feelslike_c"));
    Serial.println(" oC\n");
    
    // do nothing forevermore:
    while(true);
  }
}

String getValuesFromKey(const String response, const String sKey)
{ 
  String sKey_ = sKey;
  
  sKey_ = "\"" + sKey + "\":";
  
  char key[sKey_.length()];
  
  sKey_.toCharArray(key, sizeof(key));
  
  int keySize = sizeof(key)-1;
    
  String result = NULL;
  
  int n = response.length();
  
  for(int i=0; i < (n-keySize-1); i++)
  {
    char c[keySize];
    
    for(int k=0; k<keySize; k++)
    {
      c[k] = response.charAt(i+k);
    }
        
    boolean isEqual = true;
    
    for(int k=0; k<keySize; k++)
    {
      if(!(c[k] == key[k]))
      {
        isEqual = false;
        break;
      }
    }
    
    if(isEqual)
    {     
      int j= i + keySize + 1;
      while(!(response.charAt(j) == ','))
      {
        result += response.charAt(j);        
        j++;
      }
      
      //Remove char '"'
      result.replace("\"","");
      break;
    }
  }
  
  return result;
}
