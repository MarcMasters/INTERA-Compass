/////////////////////////////////////////////////////////////////////////////
// Paquetes
/////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "M5Stack.h"
#include "M5_BMM150.h"
#include "M5_BMM150_DEFS.h"
#include "Preferences.h"
#include "math.h"
#include <stdexcept> // Para lanzar excepciones

/////////////////////////////////////////////////////////////////////////////
// Variables
/////////////////////////////////////////////////////////////////////////////

// Variables de la tasa de refresco
unsigned long prevMillis = 0;
const long LCDinterval = 100;       // Intervalo de refresco LCD

// Selector de interfaz
int interfaz = 0;

// Variables del suavizado
bool suavizado_ponderado = false;   // Método de suavizado
int longitud_buffer = 50;           // Longitud de los buffers por defecto
float alpha = 0.7;                  // Factor de suavizado por defecto
float datos_anteriores[2] = {0.0f, 0.0f};

bool updated = false;
bool copia_ponderado = suavizado_ponderado;
int copia_len = longitud_buffer;
float copia_alpha = alpha;

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

////////////////////////////////////////////////////////////////////////////////////////////////
//                                         Calibración                                        //
////////////////////////////////////////////////////////////////////////////////////////////////

//
// Calibración en infinito
//

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

void invocar_calibracion_infinito(){
  img.fillSprite(0);
  img.drawCentreString("Flip + rotate core calibration", 160, 110, 4);
  img.pushSprite(0, 0);
  bmm150_calibrate(10000);
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

class CircularBuffer {
private:
    float* values;  // Puntero a array dinámico
    int head;       // Índice donde se almacenará el próximo valor
    int tail;       // Índice del valor más viejo
    int length;     // Tamaño del buffer
    int size;       // Número de elementos almacenados actualmente

public:
    // Constructor: Inicializa el buffer con el tamaño especificado
    CircularBuffer(int len) : head(0), tail(0), length(len), size(0) {
        if (len <= 0) {
            throw std::invalid_argument("El tamaño del buffer debe ser mayor a 0");
        }
        values = new float[len];         // Reserva memoria dinámica
    }

    // Destructor: Libera la memoria reservada. IMPORTANTE
    ~CircularBuffer() {
        delete[] values;
    }

    // Método para agregar un valor al buffer
    void nuevoValor(float value) {
        values[head] = value;
        head = (head + 1) % length;

        if (size == length) {                 
            tail = (tail + 1) % length; // Si el buffer está lleno, mueve la cola para sobrescribir el más viejo
        } else {
            size++;                     // Indica que hay un elemento más
        }
    }

    // Obtener el número de elementos almacenados
    int elementos() const {
        return size;
    }

    // Obtener la longitud del buffer
    int longitud() const {
        return length;
    }

    // Media de los valores del buffer
    float media(){
      if (size == 0){
        return 0;
      }
      float sum = 0;
      for (int i = 0, indice = tail; i < size; i++, indice = (indice + 1)%length){
        sum += values[indice];
      }
      return sum / size;
    }
};

// Buffers circulares
CircularBuffer buffer_x(longitud_buffer);
CircularBuffer buffer_y(longitud_buffer);

void suavizado_bufferCircular_array(CircularBuffer& buffer1, CircularBuffer& buffer2, float* arr_valores){
  buffer1.nuevoValor(arr_valores[0]);
  buffer2.nuevoValor(arr_valores[1]);

  arr_valores[0] = buffer1.media();
  arr_valores[1] = buffer2.media();
}

int cambiar_longitud_buffer(int len){
  if ((len-10) <= 0){
    len = 100;
  }else{
    len -= 10;
  }
  return len;
}

//
// Suavizado ponderado
//

// Funcion para suavizar un valor
float suavizar_valor(float valor_actual, float valor_anterior, float alpha){
  return  alpha * valor_anterior + (1 - alpha) * valor_actual;
}

// Funcion para suavizar un array
void suavizado_ponderado_array(float* arr_actual, float* arr_anterior, int len, float alpha){
  for (int i = 0; i < len; i++){
    arr_actual[i] = suavizar_valor(arr_actual[i],arr_anterior[i],alpha);
  }
} 

// Funcion para cambiar el factor de suavizado
float cambiar_factor_suavizado(float alpha){
  if ((alpha - 0.1) < 0){
    alpha = 0.9;
  }else{
    alpha -= 0.1;
  }
  return alpha;
}

//
// Gestión del suavizado
//

void metodo_buffer(int len, CircularBuffer& buffer1, CircularBuffer& buffer2){

  // Inicializar buffers con el tamaño especificado
  buffer1 =  CircularBuffer(len);
  buffer2 =  CircularBuffer(len);
}

void metodo_ponderado(CircularBuffer& buffer1, CircularBuffer& buffer2){
  
  // Reiniciar los buffers con la longitud mínima permitida para que ocupen poco
  buffer1 = CircularBuffer(1);
  buffer2 = CircularBuffer(1);
}

void cambiar_parametro_suavizado(bool ponderado,float& alpha,int& len){
  if(ponderado){
    alpha = cambiar_factor_suavizado(alpha);
  }else{
    len = cambiar_longitud_buffer(len);
  }
}

void salir_menu_suavizado(bool& ponderado, bool copia_ponderado, float& alpha, float copia_alpha, int& len, int copia_len,
CircularBuffer& buffer1, CircularBuffer& buffer2){

  if (ponderado != copia_ponderado){
    ponderado = copia_ponderado;
    if (ponderado){
      alpha = copia_alpha;
      metodo_ponderado(buffer1,buffer2);
    }else{
      len = copia_len;
      metodo_buffer(len,buffer1,buffer2);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
// Interfaz
/////////////////////////////////////////////////////////////////////////////

uint16_t *colors;
uint16_t darkTheme[] = {TFT_BLACK,TFT_WHITE,TFT_YELLOW,TFT_PINK};
uint16_t lightTheme[] = {TFT_WHITE,TFT_BLACK,TFT_ORANGE,TFT_BLUE};

void pickColorTheme(String colorTheme){
  // Selección de modo (claro / oscuro)
  if (colorTheme == "DARK"){
    colors = darkTheme;
  }
  else if (colorTheme == "LIGHT"){
    colors = lightTheme;
  }
  else{
    Serial.printf("Modo de color no válido.");
    return;
  }
  // Color de fondo
  M5.Lcd.fillScreen(colors[0]);
}

void setHeadingStr(float heading, int xPos, int yPos, uint16_t *colors, bool wrong=false){
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextDatum(MC_DATUM);
  if (wrong){
    M5.Lcd.setTextColor(TFT_RED);
  }
  else{
    M5.Lcd.setTextColor(colors[2]);
  }
  
  int headingRound = round(heading);
  String heading_str = String(headingRound)+(char)167;

  M5.Lcd.drawString(heading_str, xPos, yPos);
}

void drawHeading(float head_dir, int xPos, int yPos, int centerH, uint16_t *colors, bool wrongFlag=false){
  // Dibujar marco
  int w = 70; int h = 32;
  int xFramePos = xPos - w / 2; int yFramePos = yPos - h / 2;
  M5.Lcd.fillRoundRect(xFramePos, yFramePos, w, h, 7, colors[1]);

  // Dibujar pequeño triángulo
  int x1 = centerH-5; int x2 = centerH+5; int x3 = centerH;
  int y1 = yPos+h/2;  int y2 = y1;        int y3 = y1+6;
  M5.Lcd.fillTriangle(x1,y1,x2,y2,x3,y3,colors[1]);

  // Refrescar numeros
  w -= 10; h -= 10;
  xFramePos = xPos - w / 2; yFramePos = yPos - h / 2;
  M5.Lcd.fillRoundRect(xFramePos, yFramePos, w, h, 5, colors[0]);
  // Escribir dirección (int [0-360]º)
  setHeadingStr(head_dir, xPos, yPos, colors, wrongFlag);
}

void drawArrow(int centerH, int centerV, uint16_t *colors){
  centerV += 20;
  const short size = 8;
  const short size2 = 3;

  // Coordenadas del triángulo grande
  int x10 = centerH-size;
  int y10 = centerV;

  int x20 = centerH+size;
  int y20 = centerV;

  int x30 = centerH;
  int y30 = centerV-2*size;

  // Dibujo
  M5.Lcd.fillTriangle(x10, y10, x20, y20, x30, y30, colors[2]);

  // Coordenadas triángulo pequeño izq.
  int x11 = x10;
  int y11 = y10;

  int x21 = centerH;
  int y21 = y20;

  int x31 = centerH-size-size2;
  int y31 = centerV+size2+2;

  // Dibujo 
  M5.Lcd.fillTriangle(x11, y11, x21, y21, x31, y31, colors[2]);

  // Coordenadas triángulo pequeño dcha.
  int x12 = x20;
  int y12 = y20;

  int x22 = x21;
  int y22 = y21;

  int x32 = centerH+size+size2;
  int y32 = y31;

  // Dibujo
  M5.Lcd.fillTriangle(x12, y12, x22, y22, x32, y32, colors[2]);

}

void rotatePixels(float x, float y, float cx, float cy, float rotationAngle, float rotatedCoords[]){
  rotationAngle = -rotationAngle;
  rotatedCoords[0] = cos(rotationAngle * (M_PI / 180.0)) * (x-cx) - sin(rotationAngle * (M_PI / 180.0)) * (y-cy) + cx;
  rotatedCoords[1] = sin(rotationAngle * (M_PI / 180.0)) * (x-cx) + cos(rotationAngle * (M_PI / 180.0)) * (y-cy) + cy;
};

void drawLinesAndIndicators(int centerH, int centerV, float head_dir, uint16_t *colors){
  // Offset en Y
  centerV += 20;

  // Array que almacena las coordenadas x,y rotadas para su uso
  float rotatedCoords[2];

  // Radio exterior y líneas pequeñas
  int r1 = 95; int r2 = 88;

  // Sustituyo el ángulo (head_direction) por i para dibujar todas las líneas automáticamente
  for (int i=0; i<360; i+=5){
    float x1 = centerH + sin(i * (M_PI / 180.0)) * r1;
    float y1 = centerV - cos(i * (M_PI / 180.0)) * r1;

    float x2 = centerH + sin(i * (M_PI / 180.0)) * r2;
    float y2 = centerV - cos(i * (M_PI / 180.0)) * r2;

    rotatePixels(x1,y1, centerH, centerV, head_dir, rotatedCoords);
    float x1Rot = rotatedCoords[0]; float y1Rot = rotatedCoords[1];

    rotatePixels(x2,y2, centerH, centerV, head_dir, rotatedCoords);
    float x2Rot = rotatedCoords[0]; float y2Rot = rotatedCoords[1];

    M5.Lcd.drawLine(x1Rot,y1Rot,x2Rot,y2Rot,colors[1]);
  }

  // Líneas medianas
  int r3 = 81;
  for (int i=0; i<360; i+=10){
    float x1 = centerH + sin(i * (M_PI / 180.0)) * r1;
    float y1 = centerV - cos(i * (M_PI / 180.0)) * r1;

    float x3 = centerH + sin(i * (M_PI / 180.0)) * r3;
    float y3 = centerV - cos(i * (M_PI / 180.0)) * r3;

    rotatePixels(x1,y1, centerH, centerV, head_dir, rotatedCoords);
    float x1Rot = rotatedCoords[0]; float y1Rot = rotatedCoords[1];

    rotatePixels(x3,y3, centerH, centerV, head_dir, rotatedCoords);
    float x3Rot = rotatedCoords[0]; float y3Rot = rotatedCoords[1];
  
    M5.Lcd.drawLine(x1Rot,y1Rot,x3Rot,y3Rot,colors[1]);
  }

  // Líneas grandes
  int r4 = 74;
  for (int i=0; i<360; i+=30){
    float x1 = centerH + sin(i * (M_PI / 180.0)) * r1;
    float y1 = centerV - cos(i * (M_PI / 180.0)) * r1;

    float x4 = centerH + sin(i * (M_PI / 180.0)) * r4;
    float y4 = centerV - cos(i * (M_PI / 180.0)) * r4;
  
    rotatePixels(x1,y1, centerH, centerV, head_dir, rotatedCoords);
    float x1Rot = rotatedCoords[0]; float y1Rot = rotatedCoords[1];

    rotatePixels(x4,y4, centerH, centerV, head_dir, rotatedCoords);
    float x4Rot = rotatedCoords[0]; float y4Rot = rotatedCoords[1];

    M5.Lcd.drawLine(x1Rot,y1Rot,x4Rot,y4Rot,colors[1]);
  }

  // NSEO y números
  int r5 = 60; String indicator;
  for (int i=0; i<360; i+=30){
    float x5 = centerH + sin(i * (M_PI / 180.0)) * r5;
    float y5 = centerV - cos(i * (M_PI / 180.0)) * r5;

    M5.Lcd.setTextSize(1);
    switch (i) {
      case 0:
        M5.Lcd.setTextSize(2);
        indicator = "N";
        break;
      case 30:
        indicator = "NNE";
        break;
      case 60:
        indicator = "ENE";
        break;
      case 90:
        M5.Lcd.setTextSize(2);
        indicator = "E";
        break;
      case 120:
        indicator = "ESE";
        break;
      case 150:
        indicator = "SSE";
        break;
      case 180:
        M5.Lcd.setTextSize(2);
        indicator = "S";
        break;
      case 210:
        indicator = "SSO";
        break;
      case 240:
        indicator = "OSO";
        break;
      case 270:
        M5.Lcd.setTextSize(2);
        indicator = "O";
        break;
      case 300:
        indicator = "ONO";
        break;
      case 330:
        indicator = "NNO";
        break;
      default:
        indicator = String(i/10);
        break;
    }
    
    rotatePixels(x5,y5, centerH, centerV, head_dir, rotatedCoords);
    float x5Rot = rotatedCoords[0]; float y5Rot = rotatedCoords[1];

    M5.Lcd.setTextColor(colors[3]);
    M5.Lcd.drawString(indicator,x5Rot,y5Rot,2);
    // M5.Lcd.drawNumber(i/10,x5,y5);
  }
}

void setCompassUI(float head_dir, uint16_t *colors, bool wrongFlag=false){
  // Coordenadas del centro de la pantalla
  float lcdCenterH = M5.Lcd.width()/2;
  float lcdCenterV = M5.Lcd.height()/2;

  // Refrescar píxeles referentes a la brújula
  M5.Lcd.fillCircle(lcdCenterH, lcdCenterV+20, 97, colors[0]);

  // Dibujar líneas de dirección y textos de dirección
  drawLinesAndIndicators(lcdCenterH, lcdCenterV, head_dir, colors);

  // Dibujar dirección con marco
  drawHeading(head_dir,lcdCenterH,20,lcdCenterH, colors, wrongFlag);

  // Dibujar flecha estática
  drawArrow(lcdCenterH,lcdCenterV, colors);
}

void setSettingsUI(bool smoothBuffer, float smoothFactor, float bufferLen){
  // Coordenadas del centro de la pantalla
  float lcdCenterH = M5.Lcd.width()/2;
  float lcdCenterV = M5.Lcd.height()/2;

  // Limpieza de la pantalla
  M5.Lcd.fillScreen(TFT_BLACK);
  
  // Título
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString("AJUSTES DE SUAVIZADO",lcdCenterH,20);

  int verticalOffset = 30;
  // Método y Parámetro
  M5.Lcd.setTextDatum(ML_DATUM);
  M5.Lcd.drawString("METODO:", 20,lcdCenterV-verticalOffset);
  M5.Lcd.drawString("PARAMETRO:",20,lcdCenterV);

  if (smoothBuffer){
    M5.Lcd.setTextColor(TFT_YELLOW);
    M5.Lcd.drawString("PONDERADO", lcdCenterH,lcdCenterV-verticalOffset);
    M5.Lcd.drawString(String(smoothFactor), lcdCenterH,lcdCenterV);
  }
  else{
    M5.Lcd.setTextColor(TFT_YELLOW);
    M5.Lcd.drawString("BUFFER",lcdCenterH,lcdCenterV-verticalOffset);
    M5.Lcd.drawString(String(bufferLen), lcdCenterH,lcdCenterV);
  }
  
  // Textos indicativos de los botones
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setTextColor(TFT_PINK);
  M5.Lcd.drawString("btnB",lcdCenterH,lcdCenterV+80);
  M5.Lcd.drawString("PARAMETRO",lcdCenterH,lcdCenterV+100);

  M5.Lcd.drawString("btnA",lcdCenterH-110,lcdCenterV+80);
  M5.Lcd.drawString("METODO",lcdCenterH-110,lcdCenterV+100);

  M5.Lcd.drawString("btnC",lcdCenterH+110,lcdCenterV+80);
  M5.Lcd.drawString("VOLVER",lcdCenterH+110,lcdCenterV+100);
}

// Menu de calibración avanzado

/*
Este menú saltará cuando se pulse el botón B en la pantalla principal:

- Si se pulsa A se cambiará de método de suavizado (buffer circular o ponderado)
- Si se pulsa B el factor de suavizado o el tamaño del buffer (depende del método activo) cambiará
- Si se pulsa C se volverá a la pantalla principal
*/

void menu_calibracion_avanzado(bool& ponderado, float& alpha, int& len, CircularBuffer& buffer1, CircularBuffer& buffer2){
  if (!updated){
    bool copia_ponderado = suavizado_ponderado;
    int copia_len = longitud_buffer;
    float copia_alpha = alpha;
  }

  // Interfaz
  setSettingsUI(copia_ponderado, copia_alpha, copia_len);

  if (M5.BtnA.wasPressed()) {
    copia_ponderado = !copia_ponderado;
  }
  if (M5.BtnB.wasPressed()) {
    cambiar_parametro_suavizado(copia_ponderado, copia_alpha, copia_len);
  }
  if (M5.BtnC.wasPressed()) {
    salir_menu_suavizado(ponderado,copia_ponderado,alpha,copia_alpha,len,copia_len,
    buffer1,buffer2);
    interfaz = 0;
    updated = false;
    M5.Lcd.fillScreen(TFT_BLACK);
  }
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

  if (bmm150_initialization() != BMM150_OK) {
    img.fillSprite(0);  // Fill the whole sprite with defined colour.
                        
    img.drawCentreString("BMM150 init failed", 160, 110,
                         4);  // Use font 4 in (160,110)draw string.
                              
    img.pushSprite(
      0,
      0);  // Push the sprite to the TFT at 0, 0.  
    for (;;) {
      delay(100);  // delay 100ms.
    }
  }

  bmm150_offset_load();

  pickColorTheme("DARK");
}

/////////////////////////////////////////////////////////////////////////////
// Loop
/////////////////////////////////////////////////////////////////////////////

void loop() {
  M5.update();
  bmm150_read_mag_data(&dev);

  float datos[] = {dev.data.x - mag_offset.x,dev.data.y - mag_offset.y};

  if (suavizado_ponderado){
    suavizado_ponderado_array(datos,datos_anteriores,2,alpha);
  }else{
    suavizado_bufferCircular_array( buffer_x, buffer_y, datos);
  }

  datos_anteriores[0] = datos[0];
  datos_anteriores[1] = datos[1];

  float head_dir = atan2(datos[0], datos[1]) * 180.0 / M_PI;
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

  // Serial.printf("MAG X : %.2f \nMAG Y : %.2f \nMAG Z : %.2f \n", datos[0],
  //               datos[1], datos[2]);

  // // Serial.printf("MAG X : %.2f \nMAG Y : %.2f \nMAG Z : %.2f \n", xMean, yMean, zMean);

  // Serial.printf("MID X : %.2f \nMID Y : %.2f \nMID Z : %.2f \n",
  //               mag_offset.x, mag_offset.y, mag_offset.z);

  // Serial.printf("MAG X : %.2f \nMAG Y : %.2f \nMAG Z : %.2f \n", xMean, yMean, dev.data.z);

  // Serial.printf("MID X : %.2f \nMID Y : %.2f \nMID Z : %.2f \n",
  //               mag_offset.x, mag_offset.y, mag_offset.z);


  // Serial.printf("\n\n\n\n\n");

  // ---------------------------------------------------------------------------------

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
    switch (interfaz)
    {
    case 0:
      // Se activa si mantienes C y pulsas A
      if (M5.BtnB.wasPressed() && M5.BtnC.isPressed()) {
        invocar_calibracion_infinito();
      }

      // Control del aspecto de la brújula
      if (M5.BtnA.isPressed()){
        pickColorTheme("LIGHT");
      }
      if (M5.BtnA.wasReleased()){
        pickColorTheme("DARK");
      }

      if (M5.BtnB.isPressed()){
        interfaz = 1;
      }
      if (M5.BtnC.isPressed()){
        Serial.printf("AAAAAAAAAAAAAAAAA");
        interfaz = 2;
      }

      // Interfaz de la brújula
      setCompassUI(head_dir, colors, false);
      break;
    case 1:
      // Control del menú de ajustes
      menu_calibracion_avanzado(suavizado_ponderado, alpha, longitud_buffer, buffer_x, buffer_y);
      break;
    case 2:
      // Interfaz de la elección de dirección
      break;
    }
  }

  delay(10);
}