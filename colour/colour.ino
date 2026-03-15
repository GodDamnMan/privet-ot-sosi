int redPin = 5;
int greenPin = 6;
int bluePin = 7;
const int ldrPin = A0;
const int tresh = 100;
// ТВОИ ЗНАЧЕНИЯ ИЗ КАЛИБРОВКИ
int rWhite = 159, rBlack = 938; 
int gWhite = 507, gBlack = 1007;
int bWhite = 433, bBlack = 1012;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  digitalWrite(redPin, HIGH); // Выкл (анод)
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, HIGH);
  
  Serial.begin(115200);
}

void loop() {
  // 1. Считываем отражение
  int rRaw = getReading(redPin);
  int gRaw = getReading(greenPin);
  int bRaw = getReading(bluePin);

  // 2. Масштабируем (инвертируем, чтобы 255 был самым ярким)
  int r = map(rRaw, rBlack, rWhite, 0, 255);
  int g = map(gRaw, gBlack, gWhite, 0, 255);
  int b = map(bRaw, bBlack, bWhite, 0, 255);

  // Ограничиваем диапазон
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);

  // 3. Вывод данных
  Serial.print("RGB: "); Serial.print(r); Serial.print("/");
  Serial.print(g); Serial.print("/"); Serial.println(b);


  // 4. Логика распознавания
  if (r < tresh && b < tresh && g < tresh) Serial.println("---I look at heaven---"); 
  else if ( < r <)
  else if (r > g && r > b) Serial.println("--- КРАСНЫЙ ---");
  else if (g > r && g > b) Serial.println("--- СИНИЙ ---");
  else if (b > r && b > g) Serial.println("--- ЗЕЛЕНЫЙ ---");
  
  delay(100);
}

int getReading(int pin) {
  digitalWrite(pin, LOW); // Вкл (анод)
  delay(100); 
  int val = analogRead(ldrPin);
  digitalWrite(pin, HIGH); // Выкл
  return val;
}
