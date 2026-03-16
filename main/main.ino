#define DATA_PIN 15
#define RED 18
#define BLUE 17
#define GREEN 16
#define CALIBRATION 1
#define DEBUG 0

// ----------------------------------- TYPES -----------------------------------

struct RGB {
    int r;
		int g;
		int b; 
};

struct HSV {
    int h;
		int s;
		int v; 
};


// ----------------------------------- VARIABLES -----------------------------------

struct RGB white_calib = {1000, 1000, 1000};
struct RGB black_calib = {4096, 4096, 4096};

struct RGB rgb = {0, 0, 0};
struct HSV hsv = {0, 0, 0};

bool red, blue, green;




// ----------------------------------- COLOR METHODS -----------------------------------

void print_rgb(struct RGB* rgb){
	Serial.print("R: "); Serial.print(rgb->r);
  Serial.print(", G: "); Serial.print(rgb->g);
  Serial.print(", B: "); Serial.println(rgb->b);
}


void print_hsv(struct HSV* hsv){
	Serial.print("H: "); Serial.print(hsv->h);
  Serial.print(", S: "); Serial.print(hsv->s);
  Serial.print(", V: "); Serial.println(hsv->v);
}

const char* hsv_to_color_name(HSV *hsv) {
    if (hsv->v < 15) {
        return "черный";
    }
    
    if (hsv->s < 20) {
        if (hsv->v < 30) return "темно-серый";
        if (hsv->v < 60) return "серый";
        if (hsv->v < 80) return "светло-серый";
        return "белый";
    }
    
    
    float hue = hsv->h;
    
    if ((hue >= 0 && hue < 20) || (hue >= 300 && hue <= 360)) {
        return "красный";
    }
    else if (hue >= 20 && hue < 50) {
        return "оранжевый";
    }
    else if (hue >= 50 && hue < 80) {
        return "желтый";
    }
    else if (hue >= 80 && hue < 180) {
        return "зеленый";
    }
    else if (hue >= 180 && hue < 300) {
        return "синий";
    }
    
    return "неизвестный";
}


void rgb_2_hsv(struct RGB* rgb, struct HSV* hsv) {
    float r = rgb->r / 255.0f;
    float g = rgb->g /255.0f;
    float b = rgb->b /255.0f;

    float maxVal = std::max({r, g, b});
    float minVal = std::min({r, g, b});
    float delta = maxVal - minVal;
		
		if (DEBUG) {
			Serial.print(r); Serial.print(" "); Serial.print(g); Serial.print(" "); Serial.println(b);
			Serial.print(maxVal); Serial.print(" "); Serial.print(minVal); Serial.print(" "); Serial.println(delta);
		}
		
		// Value
    hsv->v = maxVal * 100; 

    // Saturation
    if (maxVal > 0.0f) {
        hsv->s = delta / maxVal * 100;
    } else {
        hsv->s = 0;
    }

    // Hue
    if (delta > 0.0f) {
        if (maxVal == r) {
            hsv->h = 60.0f * (fmod(((g - b) / delta), 6.0f));
        } else if (maxVal == g) {
            hsv->h = 60.0f * (((b - r) / delta) + 2.0f);
        } else if (maxVal == b) {
            hsv->h = 60.0f * (((r - g) / delta) + 4.0f);
        }
        
        if (hsv->h < 0.0f) {
            hsv->h += 360.0f;
        }
    } else {
        hsv->h = 0.0f;
    }
}


// ----------------------------------- LED -----------------------------------

void state_to_bools(int state){
	red = state & 1;
	blue = state & 2;
	green = state & 4;
}

void change_led(int state){
	state_to_bools(state);
	digitalWrite(RED, !red);
	digitalWrite(BLUE, !blue);
	digitalWrite(GREEN, !green);
}

int get_value(){
	int value;

	for(int i = 0; i < 5; i++) {
        delay(3);
        value += analogRead(DATA_PIN);
    }
    
  return value / 5;
}


// ----------------------------------- CALIBRATION -----------------------------------

void calibrate_white() {
    change_led(1);
    white_calib.r = get_value();
    change_led(2);
    white_calib.b = get_value();
    change_led(4);
    white_calib.g = get_value();
		change_led(0);
    
}

void calibrate_black() {
    change_led(1);
    black_calib.r = get_value();
    change_led(2);
    black_calib.b = get_value();
    change_led(4);
    black_calib.g = get_value();
		change_led(0);
}

// ----------------------------------- GET COLOR -----------------------------------

void get_color(struct RGB* color) {
  
    change_led(1);
    int raw_r = get_value();
    change_led(2);
    int raw_b = get_value();
    change_led(4);
    int raw_g = get_value();
		change_led(0);
    
    int r_norm, g_norm, b_norm;
   
    r_norm = 255 * (raw_r - black_calib.r) / (white_calib.r - black_calib.r);
    b_norm = 255 * (raw_b - black_calib.b) / (white_calib.b - black_calib.b);
    g_norm = 255 * (raw_g - black_calib.g) / (white_calib.g - black_calib.g);
    
    color->r = max(min(r_norm, 255), 0);
    color->g = max(min(g_norm, 255), 0);
    color->b = max(min(b_norm, 255), 0);
}




// ----------------------------------- MAIN -----------------------------------

void setup(){
	pinMode(DATA_PIN, INPUT);
	pinMode(RED, OUTPUT);
	pinMode(BLUE, OUTPUT);
	pinMode(GREEN, OUTPUT);
	Serial.begin(9600);

	Serial.print("\n\n--------------------------------------------\n");
	
	if (CALIBRATION) {
		Serial.println("calibrating white in 10 sec");
		delay(10000);
		calibrate_white();
		Serial.print("white - R: "); Serial.print(white_calib.r);
		Serial.print(" G: "); Serial.print(white_calib.g);
		Serial.print(" B: "); Serial.println(white_calib.b);

		Serial.println("calibrating black in 10 sec");
		delay(10000);
		calibrate_black();
		Serial.print("black - R: "); Serial.print(black_calib.r);
		Serial.print(" G: "); Serial.print(black_calib.g);
		Serial.print(" B: "); Serial.println(black_calib.b);

		Serial.println("--------------------------------------------");
		delay(5000);
	}
}

void loop(){

	get_color(&rgb);
	rgb_2_hsv(&rgb, &hsv);
	print_rgb(&rgb);
	print_hsv(&hsv);
	Serial.println(hsv_to_color_name(&hsv));
	Serial.println();
	
	delay(5000);
}
