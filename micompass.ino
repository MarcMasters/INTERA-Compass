/////////////////////////////////////////////////////////////////////////////
// Paquetes
/////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "M5Stack.h"
#include "M5_BMM150.h"
#include "M5_BMM150_DEFS.h"
#include "Preferences.h"
#include "math.h"

/////////////////////////////////////////////////////////////////////////////
// Funciones básicas magnetómetro
/////////////////////////////////////////////////////////////////////////////

Preferences prefs;

struct bmm150_dev dev;
bmm150_mag_data mag_offset;  // Compensation magnetometer float data storage
bmm150_mag_data mag_max;
bmm150_mag_data mag_min;
TFT_eSprite img = TFT_eSprite(&M5.Lcd);

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len) {
  if (M5.I2C.readBytes(
        dev_id, reg_addr, len,
        read_data)) {  // Check whether the device ID, address, data exist.
    return BMM150_OK;
  } else {
    return BMM150_E_DEV_NOT_FOUND;
  }
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len) {
  if (M5.I2C.writeBytes(dev_id, reg_addr, read_data,
                        len)) {  // Writes data of length len to the specified
                                 // device address.
    return BMM150_OK;
  } else {
    return BMM150_E_DEV_NOT_FOUND;
  }
}

int8_t bmm150_initialization() {
  int8_t rslt = BMM150_OK;

  dev.dev_id = 0x10;           // Device address setting.  
  dev.intf = BMM150_I2C_INTF;  // SPI or I2C interface setup. 
  dev.read = i2c_read;         // Read the bus pointer.  
  dev.write = i2c_write;       // Write the bus pointer. 
  dev.delay_ms = delay;

  // Set the maximum range range
  mag_max.x = -2000;
  mag_max.y = -2000;
  mag_max.z = -2000;

  // Set the minimum range
  mag_min.x = 2000;
  mag_min.y = 2000;
  mag_min.z = 2000;

  rslt = bmm150_init(&dev);  // Memory chip ID.
  dev.settings.pwr_mode = BMM150_NORMAL_MODE;
  rslt |= bmm150_set_op_mode(
    &dev);  // Set the sensor power mode.
  dev.settings.preset_mode = BMM150_PRESETMODE_ENHANCED;
  rslt |= bmm150_set_presetmode(
    &dev);  // Set the preset mode of .
  return rslt;
}

void bmm150_offset_save() {  // Store the data. 
  prefs.begin("bmm150", false);
  prefs.putBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
  prefs.end();
}

void bmm150_offset_load() {  // load the data. 
  if (prefs.begin("bmm150", true)) {
    prefs.getBytes("offset", (uint8_t *)&mag_offset,
                   sizeof(bmm150_mag_data));
    prefs.end();
    Serial.println("bmm150 load offset finish....");
  } else {
    Serial.println("bmm150 load offset failed....");
  }
}

void bmm150_calibrate(uint32_t calibrate_time) {  // bbm150 data calibrate.  
  uint32_t calibrate_timeout = 0;

  calibrate_timeout = millis() + calibrate_time;
  Serial.printf("Go calibrate, use %d ms \r\n",
                calibrate_time);  // The serial port outputs formatting
                                  // characters. 
  Serial.printf("running ...");

  while (calibrate_timeout > millis()) {
    bmm150_read_mag_data(&dev);  // read the magnetometer data from
                                 // registers.  
    if (dev.data.x) {
      mag_min.x = (dev.data.x < mag_min.x) ? dev.data.x : mag_min.x;
      mag_max.x = (dev.data.x > mag_max.x) ? dev.data.x : mag_max.x;
    }
    if (dev.data.y) {
      mag_max.y = (dev.data.y > mag_max.y) ? dev.data.y : mag_max.y;
      mag_min.y = (dev.data.y < mag_min.y) ? dev.data.y : mag_min.y;
    }
    if (dev.data.z) {
      mag_min.z = (dev.data.z < mag_min.z) ? dev.data.z : mag_min.z;
      mag_max.z = (dev.data.z > mag_max.z) ? dev.data.z : mag_max.z;
    }
    delay(100);
  }

  mag_offset.x = (mag_max.x + mag_min.x) / 2;
  mag_offset.y = (mag_max.y + mag_min.y) / 2;
  mag_offset.z = (mag_max.z + mag_min.z) / 2;
  bmm150_offset_save();

  Serial.printf("\n calibrate finish ... \r\n");
  Serial.printf("mag_max.x: %.2f x_min: %.2f \t", mag_max.x, mag_min.x);
  Serial.printf("y_max: %.2f y_min: %.2f \t", mag_max.y, mag_min.y);
  Serial.printf("z_max: %.2f z_min: %.2f \r\n", mag_max.z, mag_min.z);
}

/////////////////////////////////////////////////////////////////////////////
// Funciones para guardar la direccion actual
/////////////////////////////////////////////////////////////////////////////

String currentDirection = "";
String desiredDirection = "";
String undesiredDirection = "";

void save_direction(String direction) {
    // Se almacena la direccion actual
    currentDirection = direction;
}

void save_desiredDirection(String direction) {
    // Se almacena la direccion actual
    desiredDirection = direction;
}

void save_undesiredDirection(String direction) {
    // Se almacena la direccion actual
    undesiredDirection = direction;
}

/////////////////////////////////////////////////////////////////////////////
// Funcion para detectar la proximidad del usuario a las direcciones
/////////////////////////////////////////////////////////////////////////////

bool rUclose(String direction, String savedDirection, int proximityRange = 15) {
    //Se calcula un rango de proximidad en base a la direccion guardada
    //No obstante, primero debemos pasar de string a grados, inicializo las variables
    int dirDegrees = 0;
    int savedDegrees = 0;

    //Convierto direccion en grados
    if (direction == "NORTE"){dirDegrees = 0;}
    else if (direction == "ESTE"){dirDegrees = 90;}
    else if (direction == "SUR"){dirDegrees = 180;}
    else if (direction == "OESTE"){dirDegrees = 270;}

    if (savedDirection == "NORTE"){savedDegrees = 0;}
    else if (savedDirection == "ESTE"){savedDegrees = 90;}
    else if (savedDirection == "SUR"){savedDegrees = 180;}
    else if (savedDirection == "OESTE"){savedDegrees = 270;}

    // Calculo rango de proximidad
    int lowerBound = (savedDegrees - proximityRange + 360) % 360;
    int upperBound = (savedDegrees + proximityRange) % 360;

    //A continuacion, se comprueba si esta dentro del rango
    if (lowerBound < upperBound){
      return (dirDegrees >= lowerBound && dirDegrees <= upperBound);
    }
    else{
      return (dirDegrees >= lowerBound || dirDegrees <= upperBound); // Caso de cruce por 0° (ej. 330° a 30°)
    }
}

bool pitido_emitido = false;

/////////////////////////////////////////////////////////////////////////////
// Buffer circular
/////////////////////////////////////////////////////////////////////////////

#define CIRCULAR_BUFFER_LEN 100

typedef struct {
  int head;                           // Indice donde se almacena el proximo valor
  int tail;                           // Indice del valor mas viejo
  float values[CIRCULAR_BUFFER_LEN];  // Array de valores del tamaño del buffer establecido
} circular_buffer;

circular_buffer xBuffer;
circular_buffer yBuffer;

void value_clear(circular_buffer *buf);
void value_queue(circular_buffer *buf, float value);
float value_average(const circular_buffer *buf);

unsigned long prevMillis = 0;
const long LCDinterval = 100; // Intervalo de refresco LCD

void value_clear(circular_buffer *buf) {
  buf->head = 0;
  buf->tail = 0;
}

void value_queue(circular_buffer *buf, float value) {
  buf->values[buf->head] = value;  // valor en indice head

  // De esta forma, se le suma 1 hasta que se llena el buffer
  buf->head = (buf->head + 1) % CIRCULAR_BUFFER_LEN;

  // Si el buffer está lleno, se suma 1 a tail (nuevo valor mas viejo)
  if (buf->head == buf->tail) {
    buf->tail = (buf->tail + 1) % CIRCULAR_BUFFER_LEN;
  }
}

float value_average(const circular_buffer *buf) {
  int valuesCount = 0;
  float sum = 0.0f;

  // El buffer aun no ha dado la vuelta (hay valores vacios) (head >= tail)
  if (buf->head >= buf->tail) {
    // Se almacena desde tail hasta head
    for (int i = buf->tail; i < buf->head; i++) {
      sum += buf->values[i];
      valuesCount++;
    }


  } else {
    // El buffer ha dado la vuelta (head < tail)
    // Se almacena desde tail hasta el final del buffer
    for (int i = buf->tail; i < CIRCULAR_BUFFER_LEN; i++) {
      sum += buf->values[i];
      valuesCount++;
    }
    // Se almacena desde el principio del buffer hasta head
    for (int i = 0; i < buf->head; i++) {
      sum += buf->values[i];
      valuesCount++;
    }
  }
  // Devuelve la media si se ha contabilizado algun almacenamiento (si no, 0)
  return valuesCount > 0 ? (sum / valuesCount) : 0.0f;
}

/////////////////////////////////////////////////////////////////////////////
// Interfaz
/////////////////////////////////////////////////////////////////////////////

void LCDsetup() {
  M5.begin();

  // Color de fondo
  M5.Lcd.fillScreen(TFT_BLACK);

  // Dibujar un círculo en la pantalla
  int centerX = 160;
  int centerY = 120;
  int radius = 80;
  uint16_t fillColor = TFT_WHITE;   // Color de relleno del círculo

  M5.Lcd.fillCircle(centerX, centerY, radius, fillColor);

  // El texto que queremos centrar
  String textoN = "N";
  String textoE = "E";
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_GREEN);

  // Calcular la posición para centrar en X
  int screenWidth = M5.Lcd.width();
  int textX = (screenWidth - 6) / 2; // Coordenada X centrada (10 ancho estimado)

  // Calcular la posición para centrar en Y
  int screenHeight = M5.Lcd.height();
  int textY = (screenHeight - 18) / 2; // Coordenada Y centrada (18 altura estimada)
  
  // Dibujar el texto centrado
  M5.Lcd.drawString(textoN, textX, 10);
  M5.Lcd.drawString("S", textX, 210);
  M5.Lcd.drawString(textoE, 250, textY);
  M5.Lcd.drawString("O", 55, textY);
}

void setCompassNeedle(int needleX = 160, int needleY = 120){
  // Dibujar un círculo en la pantalla
  int radius = 7;
  uint16_t fillColor = TFT_RED;   // Color de relleno del círculo

  M5.Lcd.fillCircle(needleX, needleY, radius, fillColor);
}

void initCompassConfig() {
  img.setColorDepth(1);              // Set bits per pixel for colour.  
  img.setTextColor(TFT_WHITE);       // Set the font foreground colour (background
                                     // is. 
  img.createSprite(320, 240);        // Create a sprite (bitmap) of defined width
                                     // and height 
  img.setBitmapColor(TFT_WHITE, 0);  // Set the foreground and background
                                     // colour. 
}

void setHeadingStr(float heading){
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_RED);
  String heading_str = String(heading);
  M5.Lcd.drawString(heading_str, 15, 210);
}

void setTriangleCompassNeedle(float heading){
    // Punta del triangulo:

    // Cos y Sin estan invertidos por adaptacion al sistema de referencia de una brujula
    // (0 arriba y 180 abajo, a diferencia del sistema trigonométrico de 0 derecha y 180 izquierda)
    
    // M5.Width/2 + sin(heading(rad)) * (circle_radius-offset)
    float x1 = M5.Lcd.width()/2 + sin(heading * (M_PI / 180.0)) * (80-10);
    // M5.Height/2 - cos(heading(rad)) * (circle_radius-offset)
    float y1 = M5.Lcd.height()/2 - cos(heading * (M_PI / 180.0)) * (80-10);

    // Segundo vértice (desfasado -90º):
    float x2 = M5.Lcd.width()/2 + sin((heading - 90) * (M_PI / 180.0)) * (15);
    float y2 = M5.Lcd.height()/2 - cos((heading - 90) * (M_PI / 180.0)) * (15);

    // Tercer vértice (desfasado +90º):
    float x3 = M5.Lcd.width()/2 + sin((heading + 90) * (M_PI / 180.0)) * (15);
    float y3 = M5.Lcd.height()/2 - cos((heading + 90) * (M_PI / 180.0)) * (15);

    // Dibujo:
    M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_RED);
}

/////////////////////////////////////////////////////////////////////////////
// SetUp
/////////////////////////////////////////////////////////////////////////////

void setup() {
  M5.Speaker.begin();
  M5.begin(true, false, true, false);  // Init M5Core(Initialize LCD, serial port). 
                                       // M5Core
  M5.Power.begin();                    // Init Power module. 
  Wire.begin(
    21, 22,
    400000UL
  );  // Set the frequency of the SDA SCL. 

  //LCDsetup();

  if (bmm150_initialization() != BMM150_OK) {
    img.fillSprite(0);  // Fill the whole sprite with defined colour.
                        
    img.drawCentreString("BMM150 init failed", 160, 110,
                         4);  // Use font 4 in (160,110)draw string.
                              
    img.pushSprite(
      0,
      0);  // Push the sprite to the TFT at 0, 0.  
    for (;;) {
      delay(100);  // delay 100ms.  延迟100ms
    }
  }

  bmm150_offset_load();
}

/////////////////////////////////////////////////////////////////////////////
// Loop
/////////////////////////////////////////////////////////////////////////////

void loop() {
  char text_string[100];
  M5.update();  // Read the press state of the key.
  bmm150_read_mag_data(&dev);
  float head_dir = atan2(dev.data.x - mag_offset.x, dev.data.y - mag_offset.y) * 180.0 / M_PI;
  if (head_dir < 0.0) {
    head_dir += 360;
  }

  char rumbo[6] = "";
  if (head_dir >= 315.0 || head_dir <= 45.0) {
    strcpy(rumbo, "NORTE");
  } else if (head_dir >= 225.0 && head_dir <= 315.0) {
    strcpy(rumbo, "OESTE");
  } else if (head_dir >= 45.0 && head_dir <= 135.0) {
    strcpy(rumbo, "ESTE");
  } else {
    strcpy(rumbo, "SUR");
  }

  // -- Serial Monitor ---------------------------------------------------------------

  // Serial.printf("Magnetometer data, heading %.2f\n >>> Rumbo: %s \n", head_dir, rumbo);

  //Serial.printf("MAG X : %.2f \nMAG Y : %.2f \nMAG Z : %.2f \n", dev.data.x,
  //              dev.data.y, dev.data.z);

  value_queue(&xBuffer, dev.data.x);
  value_queue(&yBuffer, dev.data.y);

  float xMean = value_average(&xBuffer);
  float yMean = value_average(&yBuffer);

  // Serial.printf("MAG X : %.2f \nMAG Y : %.2f \nMAG Z : %.2f \n", xMean, yMean, dev.data.z);

  // Serial.printf("MID X : %.2f \nMID Y : %.2f \nMID Z : %.2f \n",
  //               mag_offset.x, mag_offset.y, mag_offset.z);


  // Serial.printf("\n\n\n\n\n");

  // ---------------------------------------------------------------------------------

  // img.fillSprite(0);
  // sprintf(text_string, "MAG X: %.2f", dev.data.x);
  // img.drawString(text_string, 10, 20,
  //                4);  // draw string with padding.
  // sprintf(text_string, "MAG Y: %.2f", dev.data.y);
  // img.drawString(text_string, 10, 50, 4);
  // sprintf(text_string, "MAG Z: %.2f", dev.data.z);
  // img.drawString(text_string, 10, 80, 4);
  // sprintf(text_string, "HEAD Angle: %.2f", head_dir);
  // img.drawString(text_string, 10, 110, 4);
  // img.drawCentreString("Press BtnA enter calibrate", 160, 150, 4);
  // img.pushSprite(0, 0);

  if (M5.BtnA.wasPressed()) {
    img.fillSprite(0);
    img.drawCentreString("Flip + rotate core calibration", 160, 110, 4);
    img.pushSprite(0, 0);
    bmm150_calibrate(10000);
  }

  if (M5.BtnC.wasPressed()) {
    save_direction(rumbo);
    Serial.printf("Direccion actual guardada");

    //Rectangulo para preguntar donde guardar la direccion
    M5.Lcd.fillRoundRect(10, 100, 300, 50, 5, TFT_GREEN);
    M5.Lcd.drawRoundRect(10, 100, 300, 50, 5, TFT_WHITE);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_GREEN);
    M5.Lcd.drawCentreString("¿Deseas esta direccion?", 10 + 300 / 2, 100 + 50 / 2 - 8, 1);

    bool decisionTaken = false;

    while (!decisionTaken) {
      M5.update(); // Actualizar el estado de los botones

      //Direccion deseada o no deseada
      if (M5.BtnA.wasPressed()) {
        save_desiredDirection(rumbo);
        Serial.println("Dirección marcada como deseada.");
        decisionTaken = true; // Salir del bucle
        }

      if (M5.BtnC.wasPressed()) {
        save_undesiredDirection(rumbo);
        Serial.println("Dirección marcada como no deseada.");
        decisionTaken = true; // Salir del bucle
        }

      delay(10);
    }
  }

  if (desiredDirection != ""){
    if (rUclose(rumbo,desiredDirection)){
      if (!pitido_emitido){
        M5.Speaker.tone(661, 500);
        pitido_emitido = true;
      }
    }
    else{
      pitido_emitido = false;
    }
  }

  if (undesiredDirection != ""){
    if (rUclose(rumbo,undesiredDirection)){
      if (!pitido_emitido){
        M5.Speaker.tone(440, 500);
        pitido_emitido = true;
      }
    }
    else{
      pitido_emitido = false;
    }
  }

  unsigned long currMillis = millis();
  if (currMillis - prevMillis >= LCDinterval) {
    prevMillis = currMillis;

    // Actualizar la pantalla cada 100ms
    LCDsetup();
    setHeadingStr(head_dir);
    setTriangleCompassNeedle(head_dir);
  }

  delay(10);
}