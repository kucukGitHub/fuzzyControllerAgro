#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE(*_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE(*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE(*_FIS_ARR)(FIS_TYPE*, int, _FIS_ARR_OP);

//#include "fis_header.h"
//#include "fuzzy.h"

// Number of inputs to the fuzzy inference system
const int fis_gcI = 3;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 27;
const int trigPin = 12; // Pin de salida del pulso del sensor
const int echoPin = 13; // Pin de entrada del eco del sensor
const float tanqueLleno = 20.0; // Altura en cm cuando el tanque está lleno
const float tanqueVacio = 170.0; // Altura en cm cuando el tanque está vacío
const float alturaTanque = tanqueVacio - tanqueLleno; // Altura total del tanque
const float radioTanque = 25.0;

const int switchPin = 6; 
int humedadTotal = 0; 
int contador = 0; 

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

// Setup routine runs once when you press reset:
void setup()
{
    Serial.begin(9600); 
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    // Pin mode for Input: Humedad
    pinMode(0 , INPUT);
    // Pin mode for Input: edad_plant
    pinMode(1 , INPUT);
    // Pin mode for Input: cant_sup
    pinMode(2 , INPUT);


    // initialize the Analog pins for output.
    // Pin mode for Output: lt
   pinMode(3 , OUTPUT);
   pinMode(switchPin, OUTPUT);
   
}


// Loop routine runs over and over again forever:
void loop()
{
    // Read Input: Humedad
    g_fisInput[0] = analogRead(0);
    // Read Input: edad_plant
    g_fisInput[1] = analogRead(1);
    // Read Input: cant_sup
    g_fisInput[2] = analogRead(2);
    g_fisOutput[0] = 0;
    fis_evaluate();
    // Set output vlaue: lt
    analogWrite(3 , g_fisOutput[0]);
    ////ultrasonic
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distancia = duration * 0.034 / 2;

  // tank
  float alturaAgua = tanqueVacio - distancia;
  // volumen
  float volumenAgua = volumenCilindro(radioTanque, alturaAgua);

  // Imprimir la distancia y el volumen de agua en litros
  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.print(" cm, Volumen de agua: ");
  Serial.print(volumenAgua);
  Serial.println(" litros");

  delay(1000); 

  //check humidity
  if (Serial.available() > 0) { // Si hay datos disponibles en el puerto serial
    String mensaje = Serial.readStringUntil('\n'); // Leer el mensaje hasta encontrar un salto de línea

    // Buscar el valor de humedad en el mensaje
    int posInicio = mensaje.indexOf("(") + 1;
    int posFin = mensaje.indexOf(")");
    String humedadStr = mensaje.substring(posInicio, posFin);
    int humedad = humedadStr.toInt();

    // Actualizar la suma de las humedades y el contador
    humedadTotal += humedad;
    contador++;

    // Si se han recibido 3 mensajes, calcular el promedio de humedad
    if (contador == 3) {
      float promedio = (float)humedadTotal / contador;

      // Imprimir el promedio de humedad
      Serial.print("Promedio de humedad: ");
      Serial.println(promedio);

      // Si el promedio es mayor que 90, activar el switch
      if (promedio > 90) {
        digitalWrite(switchPin, HIGH); // Activar el switch
        Serial.println("Señal de activación enviada al switch");
      } else {
        digitalWrite(switchPin, LOW); // Desactivar el switch
      }

      // Reiniciar las variables para el siguiente ciclo
      humedadTotal = 0;
      contador = 0;

      Serial.flush(); // Limpiar el buffer del puerto serial
    }
  }  
}
float volumenCilindro(float radio, float altura) {
  return PI * radio * radio * altura;
}
//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 3, 3, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 3 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 0, 20, 40 };
FIS_TYPE fis_gMFI0Coeff2[] = { 30, 50, 70 };
FIS_TYPE fis_gMFI0Coeff3[] = { 60, 80, 100 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE fis_gMFI1Coeff1[] = { 0, 0.4, 0.8 };
FIS_TYPE fis_gMFI1Coeff2[] = { 0.6, 1, 1.4 };
FIS_TYPE fis_gMFI1Coeff3[] = { 1.2, 1.6, 2 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3 };
FIS_TYPE fis_gMFI2Coeff1[] = { 0, 10, 20 };
FIS_TYPE fis_gMFI2Coeff2[] = { 18, 50, 70 };
FIS_TYPE fis_gMFI2Coeff3[] = { 65, 80, 100 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2, fis_gMFI2Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 50, 100 };
FIS_TYPE fis_gMFO0Coeff2[] = { 80, 140, 200 };
FIS_TYPE fis_gMFO0Coeff3[] = { 180, 240, 300 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0 };
int fis_gMFI1[] = { 0, 0, 0 };
int fis_gMFI2[] = { 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2};

// Output membership function set
int fis_gMFO0[] = { 0, 0, 0 };
int* fis_gMFO[] = { fis_gMFO0};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1, 1 };
int fis_gRI1[] = { 1, 2, 1 };
int fis_gRI2[] = { 1, 3, 1 };
int fis_gRI3[] = { 1, 1, 2 };
int fis_gRI4[] = { 1, 2, 2 };
int fis_gRI5[] = { 1, 3, 2 };
int fis_gRI6[] = { 1, 1, 3 };
int fis_gRI7[] = { 1, 2, 3 };
int fis_gRI8[] = { 1, 3, 3 };
int fis_gRI9[] = { 2, 1, 1 };
int fis_gRI10[] = { 2, 2, 1 };
int fis_gRI11[] = { 2, 3, 1 };
int fis_gRI12[] = { 2, 1, 2 };
int fis_gRI13[] = { 2, 2, 2 };
int fis_gRI14[] = { 2, 3, 2 };
int fis_gRI15[] = { 2, 1, 3 };
int fis_gRI16[] = { 2, 2, 3 };
int fis_gRI17[] = { 2, 3, 3 };
int fis_gRI18[] = { 3, 1, 1 };
int fis_gRI19[] = { 3, 2, 1 };
int fis_gRI20[] = { 3, 3, 1 };
int fis_gRI21[] = { 3, 1, 2 };
int fis_gRI22[] = { 3, 2, 2 };
int fis_gRI23[] = { 3, 3, 2 };
int fis_gRI24[] = { 3, 1, 3 };
int fis_gRI25[] = { 3, 2, 3 };
int fis_gRI26[] = { 3, 3, 3 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13, fis_gRI14, fis_gRI15, fis_gRI16, fis_gRI17, fis_gRI18, fis_gRI19, fis_gRI20, fis_gRI21, fis_gRI22, fis_gRI23, fis_gRI24, fis_gRI25, fis_gRI26 };

// Rule Outputs
int fis_gRO0[] = { 1 };
int fis_gRO1[] = { 1 };
int fis_gRO2[] = { 1 };
int fis_gRO3[] = { 2 };
int fis_gRO4[] = { 2 };
int fis_gRO5[] = { 2 };
int fis_gRO6[] = { 3 };
int fis_gRO7[] = { 3 };
int fis_gRO8[] = { 3 };
int fis_gRO9[] = { 1 };
int fis_gRO10[] = { 1 };
int fis_gRO11[] = { 1 };
int fis_gRO12[] = { 2 };
int fis_gRO13[] = { 2 };
int fis_gRO14[] = { 2 };
int fis_gRO15[] = { 2 };
int fis_gRO16[] = { 3 };
int fis_gRO17[] = { 3 };
int fis_gRO18[] = { 1 };
int fis_gRO19[] = { 1 };
int fis_gRO20[] = { 1 };
int fis_gRO21[] = { 1 };
int fis_gRO22[] = { 1 };
int fis_gRO23[] = { 1 };
int fis_gRO24[] = { 1 };
int fis_gRO25[] = { 1 };
int fis_gRO26[] = { 1 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13, fis_gRO14, fis_gRO15, fis_gRO16, fis_gRO17, fis_gRO18, fis_gRO19, fis_gRO20, fis_gRO21, fis_gRO22, fis_gRO23, fis_gRO24, fis_gRO25, fis_gRO26 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 100, 2, 100 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 300 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
