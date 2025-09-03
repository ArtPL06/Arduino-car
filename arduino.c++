#include <Servo.h>

#define TRIG_PIN A0
#define ECHO_PIN A1
#define SERVO_PIN 2

Servo servo;
int melhorAngulo = 90;
int maiorDistancia = 0;

int sensorEsquerdaPin = 3;
int sensorDireitaPin = 12;
int motorEsquerdoPin1 = 9;
int motorEsquerdoPin2 = 10;
int motorDireitoPin1 = 5;
int motorDireitoPin2 = 6;

int velocidadeFrente = 180;
int velocidadeCurva = 130;

void setup() {
    Serial.begin(9600);
    servo.attach(SERVO_PIN);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(motorEsquerdoPin1, OUTPUT);
    pinMode(motorEsquerdoPin2, OUTPUT);
    pinMode(motorDireitoPin1, OUTPUT);
    pinMode(motorDireitoPin2, OUTPUT);

    pinMode(sensorDireitaPin, INPUT);
    pinMode(sensorEsquerdaPin, INPUT);
}

long medirDistancia() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duracao = pulseIn(ECHO_PIN, HIGH);
    return duracao * 0.034 / 2;
}

void varredura() {
    maiorDistancia = 0;

    for (int angulo = 0; angulo <= 180; angulo += 15) {
        servo.write(angulo);
        delay(500);
        int distancia = medirDistancia();
        
        Serial.print("Ângulo: ");
        Serial.print(angulo);
        Serial.print(" - Distância: ");
        Serial.println(distancia);

        if (distancia > maiorDistancia) {
            maiorDistancia = distancia;
            melhorAngulo = angulo;
        }
    }
}

void moverFrente() {
    analogWrite(motorEsquerdoPin1, velocidadeFrente);
    analogWrite(motorEsquerdoPin2, 0);
    analogWrite(motorDireitoPin1, velocidadeFrente);
    analogWrite(motorDireitoPin2, 0);
    Serial.println("Indo pra frente");
}

void parar() {
    analogWrite(motorEsquerdoPin1, 0);
    analogWrite(motorEsquerdoPin2, 0);
    analogWrite(motorDireitoPin1, 0);
    analogWrite(motorDireitoPin2, 0);
    Serial.println("Parado");
}

void virarDireita() {
    analogWrite(motorEsquerdoPin1, velocidadeCurva);
    analogWrite(motorEsquerdoPin2, 0);
    analogWrite(motorDireitoPin1, 0);
    analogWrite(motorDireitoPin2, 0);
    Serial.println("Virando para direita");
}

void virarEsquerda() {
    analogWrite(motorEsquerdoPin1, 0);
    analogWrite(motorEsquerdoPin2, 0);
    analogWrite(motorDireitoPin1, velocidadeCurva);
    analogWrite(motorDireitoPin2, 0);
    Serial.println("Virando para esquerda");
}

void loop() {
    int sensorEsquerda = digitalRead(sensorEsquerdaPin);
    int sensorDireita = digitalRead(sensorDireitaPin);
    long distanciaFrontal = medirDistancia();

    Serial.print("Distância frontal: ");
    Serial.println(distanciaFrontal);

    if (distanciaFrontal < 20) {
        Serial.println("Obstáculo detectado! Iniciando varredura...");
        parar();
        varredura();
        Serial.print("Melhor direção: ");
        Serial.println(melhorAngulo);
        servo.write(melhorAngulo);
        
        if (melhorAngulo < 90) {
            virarDireita();
        } else {
            virarEsquerda();
        }
        delay(1000);
    } else {
        if (sensorDireita == LOW && sensorEsquerda == LOW) {
            moverFrente();
        } else if (sensorDireita == HIGH && sensorEsquerda == HIGH) {
            parar();
        } else if (sensorEsquerda == HIGH && sensorDireita == LOW) {
            virarDireita();
        } else if (sensorEsquerda == LOW && sensorDireita == HIGH) {
            virarEsquerda();
        }
    }

    delay(50);
}
