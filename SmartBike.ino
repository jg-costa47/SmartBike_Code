#include <BluetoothSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DHT.h"
#include "NewPing.h"

//____________________________________________________________________________  
//------------------------------ Check Bluetooth ----------------------------   
//____________________________________________________________________________

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth não está habilitado! 
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth Serial não está disponível. A conexão apenas está disponível no ESP32.
#endif

bool emparelhado = false;

//____________________________________________________________________________  
//------------------------------ Definição DHT11 ---------------------------- 
//____________________________________________________________________________

#define DHTPIN          23
#define DHTTYPE         DHT11
DHT dht(DHTPIN, DHTTYPE);

//____________________________________________________________________________  
//----------------------------- Definição SW-520 ----------------------------   
//____________________________________________________________________________

#define SW520_PINO      15 
#define BOTAO_PINO      5 
#define BUZZER_PINO     22  

//____________________________________________________________________________  
//---------------------------- Definição HC-SR04 ----------------------------   
//____________________________________________________________________________

#define DISTANCIA_MAXIMA    400 // 4 metros
#define PINO_TRIGGER    19
#define PINO_ECHO       21
#define PINO_TRIGGER2   5
#define PINO_ECHO2      18
#define PINO_TRIGGER3   2
#define PINO_ECHO3      4
#define VELOCIDADE_SOM_1   331.4
#define VELOCIDADE_SOM_2   0.034
//#define LEITURA_OBJETOS_LONGE 5

#define OBJETO_DETETADO  2000 // duração do sinal do buzzer

NewPing sonar(PINO_TRIGGER, PINO_ECHO, DISTANCIA_MAXIMA);
NewPing sonar3(PINO_TRIGGER3, PINO_ECHO3, DISTANCIA_MAXIMA);

//____________________________________________________________________________  
//----------------------- VARIAVEIS AUXILIARES(Teste) -----------------------   
//____________________________________________________________________________

NewPing sonar_aux3(PINO_TRIGGER3, PINO_ECHO3, DISTANCIA_MAXIMA);
NewPing sonar_aux(PINO_TRIGGER, PINO_ECHO, DISTANCIA_MAXIMA);

float duracao_aux;
float duracao_aux3;
float distancia_cm1;
float distancia_cm3;

float distancia_cm_aux;
float distancia_cm_aux3;

float humidade_aux;
float temperatura_aux;
int iteracoes_aux = 10;

//____________________________________________________________________________  
//-----------------------------VARIAVEIS GLOBAIS-----------------------------   
//____________________________________________________________________________

bool dhtOperacional = false;

bool acidenteAtivo = false;
unsigned long detecao_Acidente = 0;

bool buzzerAtivo = false;

bool estadoMudou_SW520 = false;
bool ultimoEstado_SW520 = LOW;

bool permitirAcidente = true; 

bool objetoDetetadoAnterior = false;
unsigned long tempoInicial = 0; //Timer Obj entre os 100 e os 300 metros


float som_ms;
float som_cm;

//____________________________________________________________________________  
//---------------------------- Início do Código -----------------------------   
//____________________________________________________________________________

BluetoothSerial SerialBT;
bool LigacaoBluetoothEstabelecida = false;
bool aguardaConectar = true;

bool Conectar_SmartBike() {
  bool senhaRequerida = false;

  while (true) {
    if (aguardaConectar) {
      SerialBT.println("Pretende iniciar um novo ride de SmartBike? Selecione: 'Conectar'");
      aguardaConectar = false;
      delay(2000);
    }

    if (SerialBT.available()) {
      String conectar = SerialBT.readStringUntil('\n');
      conectar.trim();

      if (conectar.equals("conectar")) {
        SerialBT.println("Pretende começar um ride de SmartBike? ('Sim' | 'Não')");
        while (true) {
          while (SerialBT.available() == 0) {
            // Aguarda pela mensagem do ciclista
          }

          String resposta = SerialBT.readStringUntil('\n');
          resposta.trim();

          if (resposta.equals("Sim")) {
            if (!senhaRequerida) {
              SerialBT.println("SmartBike Password: ****");
              senhaRequerida = true;
              delay(2000);
            }

            while (true) {
              while (SerialBT.available() == 0) {
                // Aguarda até que a password seja introduzida
              }

              String password_SmartBike = SerialBT.readStringUntil('\n');
              password_SmartBike.trim();

              if (password_SmartBike.equals("1234")) {
                SerialBT.println("Password correta. Boa viagem de SmartBike!");
                delay(2000);
                return true; // Início da viagem de Smartbike.
              } else {
                SerialBT.println("Password incorreta.");
                SerialBT.println("SmartBike Password: ****");
              }
            }
          } else if (resposta.equals("Não")) {
            SerialBT.println("Obrigado pela experiência na SmartBike.");
              delay(3000);

            // Desconectar a ligação por Bluetooth com a App
            return false;
          }
        }
      }
    }
  }
  return false; // Password incorreta
}

// Manipuladores das tarefa
TaskHandle_t tarefaHCSR04Handle;
TaskHandle_t tarefaDHT11Handle;
TaskHandle_t tarefaSW520Handle;
TaskHandle_t tarefaBluetoothHandle;

// Funções das tarefas
void tarefaHCSR04(void* parametro);
void tarefaDHT11(void* parametro);
void tarefaSW520(void* parametro);
void tarefaBluetooth(void* parametro);

// Funções Auxiliares  das tarefas
void acionarAcaoAcidente();
void atualizaDetecaoAcidente();
float validarDistancia(float distancia);
void interrupcaoAcidente();
void IRAM_ATTR interrupcaoBotao(); 


//____________________________________________________________________________  
//--------------------------------- SET-UP ----------------------------------   
//____________________________________________________________________________

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Gikabala"); 
  Serial.println("------------------Loading Bluetooth.");

  LigacaoBluetoothEstabelecida = Conectar_SmartBike(); // Verificar password da SmartBike
  dht.begin();

  // Criar tarefas
  xTaskCreatePinnedToCore(
      tarefaHCSR04,           // Função a ser executada pela tarefa HCSR04
      "TarefaHCSR04",         // Nome da tarefa
      10000,                  // Tamanho da stack (bytes)
      NULL,                   // Parâmetro da tarefa
      1,                      // Prioridade da tarefa
      &tarefaHCSR04Handle,    // Manipulador da tarefa
      0                       // Número do núcleo (0 ou 1)
  );

  xTaskCreatePinnedToCore(
      tarefaDHT11,            // Função a ser executada pela tarefa DHT11
      "TarefaDHT11",          // Nome da tarefa
      10000,                  // Tamanho da stack (bytes)
      NULL,                   // Parâmetro da tarefa
      8,                      // Prioridade da tarefa
      &tarefaDHT11Handle,     // Manipulador da tarefa
      0                       // CORE (0 ou 1)
  );

  xTaskCreatePinnedToCore(
      tarefaSW520,            // Função a ser executada pela tarefa SW520
      "MonitorOFF",           // Nome da tarefa
      10000,                  // Tamanho da stack (bytes)
      NULL,                   // Parâmetro da tarefa
      6,                      // Prioridade da tarefa
      &tarefaSW520Handle,     // Manipulador da tarefa
      0                       // CORE (0 ou 1)
  );

  xTaskCreatePinnedToCore(
      tarefaBluetooth,        // Função a ser executada pela tarefa Bluetooth
      "TarefaBluetooth",      // Nome da tarefa
      10000,                  // Tamanho da stack (bytes)
      NULL,                   // Parâmetro da tarefa
      5,                      // Prioridade da tarefa
      &tarefaBluetoothHandle, // Manipulador da tarefa
      0                       // CORE (0 ou 1)
  );

  pinMode(SW520_PINO, INPUT_PULLUP); // Sensor SW-520 como entrada com pull-up
  pinMode(BOTAO_PINO, INPUT_PULLUP); // Botão como entrada com pull-up
  pinMode(BUZZER_PINO, OUTPUT);      // Buzzer como saída

  attachInterrupt(digitalPinToInterrupt(BOTAO_PINO), interrupcaoAcidente, FALLING);
  
}

//____________________________________________________________________________  
//-------------------------------- Loop AUX ---------------------------------   
//____________________________________________________________________________

void loop() { // Loop Auxiliar 

  if (!LigacaoBluetoothEstabelecida) {
    aguardaConectar = true; // Conectar novamente 
    LigacaoBluetoothEstabelecida = false;
    
  } else {
    LigacaoBluetoothEstabelecida = true;
    LigacaoBluetoothEstabelecida = Conectar_SmartBike(); // Inicia a função de conexão
  }

  if (LigacaoBluetoothEstabelecida) {


    int sensorValue = digitalRead(SW520_PINO);
    duracao_aux = sonar_aux.ping_median(iteracoes_aux);
    distancia_cm_aux = duracao_aux * VELOCIDADE_SOM_2 / 2;
    duracao_aux3 = sonar_aux3.ping_median(iteracoes_aux);
    distancia_cm_aux3 = duracao_aux3 * VELOCIDADE_SOM_2 / 2;
    
    humidade_aux = dht.readHumidity();
    temperatura_aux = dht.readTemperature();

    Serial.print("Temperatura: ");
    Serial.print(temperatura_aux);

    Serial.print("Humididade: ");
    Serial.println(humidade_aux);
    /*
    if (sensorValue == HIGH) { 
        Serial.println("ON-State");

        delay(500);
    } else {
        Serial.println("OFF-State");
        delay(500);
    }
    */
    Serial.println(distancia_cm_aux);
    Serial.println(distancia_cm_aux3);
    delay(500);
  }  
  else{
    LigacaoBluetoothEstabelecida = false;
    Conectar_SmartBike(); // Inicia a ligação por Bluetooth com a App
  }
}
/*
____________________________________________________________________________ 
                                      NOTAS
____________________________________________________________________________ 
Objeto detetado 1 metro e meio -> DEMO 30 cm 
    - (Envio de apenas uma mensagem)

Objeto entre 1 metro e 20 cm -> DEMO 20 cm e 10 cm 
    - (Envio consecutivo de mensagens com o valor da distância)
____________________________________________________________________________ 
*/


//____________________________________________________________________________  
//--------------------------- Deteção Obstáculo -----------------------------   
//____________________________________________________________________________

float validarDistancia(float distancia) {
  if (distancia < 10 || distancia > 300) { //Distancias inferiores a 20 cm  e superiores a 3 metros, são excluidas (Na DEMO são excluídos valores inferiores a 10 cm) 
    return NAN; // Atribuir valor inválido
  }
  return distancia;
}

void tarefaHCSR04(void* parametro) {
  static float duracao;
  static float duracao3;
  static int iteracoes = 10;

  while (1) {
    if (LigacaoBluetoothEstabelecida){
      if (!acidenteAtivo){
        // Calcular a distância
        duracao = sonar.ping_median(iteracoes);
        duracao3 = sonar3.ping_median(iteracoes);

        if (dhtOperacional) {
          // Calcular a distância com base no som_cm
          distancia_cm1 = duracao * som_cm / 2;
          distancia_cm3 = duracao3 * som_cm / 2;
        } else {
          // Calcular a distância com base no VELOCIDADE_SOM_2
          distancia_cm1 = duracao * VELOCIDADE_SOM_2 / 2;
          distancia_cm3 = duracao3 * VELOCIDADE_SOM_2 / 2;
        }
        distancia_cm1 = validarDistancia(distancia_cm1);
        distancia_cm3 = validarDistancia(distancia_cm3);

        float menorDistanciaObj = min(distancia_cm1, distancia_cm3);


        if ((menorDistanciaObj > 100) && (menorDistanciaObj < 300)){
          if (!objetoDetetadoAnterior){
            tempoInicial = millis();
            objetoDetetadoAnterior = true;
          }else{
            if (millis() - tempoInicial >= 5000){
                objetoDetetadoAnterior = false;
            }
          }
        }
        else{ 
          objetoDetetadoAnterior = false;
        }
        // Atraso entre as mediCOes
        vTaskDelay(pdMS_TO_TICKS(300));
      }else {
        vTaskSuspend(NULL); // Colocar a tarefa em standby durante a interrupção
      }
    }
  }
}



// Determinar a velocidade do som calibrada pela Temperatura e Humidade.
void tarefaDHT11(void* parametro) {
  static float humidade;
  static float temperatura;
  
  while (1) {
    if (!acidenteAtivo) {
      // Ler humidade e temperatura
      humidade = dht.readHumidity();
      temperatura = dht.readTemperature();

      if (!isnan(humidade) && !isnan(temperatura)) {
        dhtOperacional = true;
        // Calcular a velocidade do som calibrada
        som_ms = VELOCIDADE_SOM_1 + (0.606 * temperatura) + (0.0124 * humidade);
        som_cm = som_ms / 10000;
      } else {
        dhtOperacional = false;
      }
      // Atraso entre as medições
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      vTaskSuspend(NULL); // Colocar a tarefa em standby durante a interrupção
    }
  }
}

//____________________________________________________________________________  
//--------------------------- Deteção Acidente ------------------------------   
//____________________________________________________________________________

void acionarAcaoAcidente() {
  acidenteAtivo = true;
  buzzerAtivo = true;
  digitalWrite(BUZZER_PINO, HIGH); // Ativar o buzzer
}

void atualizaDetecaoAcidente() {
  detecao_Acidente = millis();
}

void tarefaSW520(void* parametro) {
  while (1) {
    if (LigacaoBluetoothEstabelecida){
      int sensorSW520 = digitalRead(SW520_PINO);
      sensorSW520 = !sensorSW520;

      if (sensorSW520 != ultimoEstado_SW520) {
        ultimoEstado_SW520 = sensorSW520;
        estadoMudou_SW520 = true;
        if (sensorSW520 == HIGH) {
          atualizaDetecaoAcidente();
        }
      }

      if (sensorSW520 == HIGH && millis() - detecao_Acidente >= 10000 && estadoMudou_SW520) {
        acionarAcaoAcidente();
        
        estadoMudou_SW520 = false;
      } else if (sensorSW520 == LOW) {
        estadoMudou_SW520 = false;
        atualizaDetecaoAcidente();
      }

      vTaskDelay(pdMS_TO_TICKS(10)); // Pequeno atraso para evitar polling excessivo
    }
  }
}

// Interrupção do Acidente - Estado Falso Alamre
void interrupcaoAcidente() {
  if (!permitirAcidente) {
    acionarAcaoAcidente();
    buzzerAtivo = true;
    digitalWrite(BUZZER_PINO, HIGH);
    permitirAcidente = false; // Desativar novo acionamento do acidente
  } else { // Falso Alarme
    acidenteAtivo = false;
    buzzerAtivo = false;
    digitalWrite(BUZZER_PINO, LOW);
    atualizaDetecaoAcidente();
    vTaskResume(tarefaHCSR04Handle);
    vTaskResume(tarefaDHT11Handle);
    permitirAcidente = true; // Permitir novo acionamento do acidente
  }
}

//____________________________________________________________________________  
//--------------------------- Comunicação MApp  -----------------------------   
//____________________________________________________________________________

void tarefaBluetooth(void* parametro) {

  static bool ultimoEstadoBuzzer = false; 
  bool mensagemObjetoDetetado = false; // Variável para controlar o envio da mensagem de objeto a 1 metro e meio.

  while (1) {
    if (LigacaoBluetoothEstabelecida){
      if (!acidenteAtivo) {
        String valores = ""; 
        float menorDistancia = min(distancia_cm1, distancia_cm3);
        if (menorDistancia <= 20){ // Objeto detetado a menos de 1 metro (Na demo 20 cm) -> envio dos dados consecutivos da distância.
          valores += "Distância: " + String(menorDistancia) + " cm" + "\n";
      
        }

        if (objetoDetetadoAnterior && !mensagemObjetoDetetado) { // Envio da mensagem Objeto detetado.
          SerialBT.println("Objeto detectado a pelo menos 1 metro.");
          mensagemObjetoDetetado = true;
          digitalWrite(BUZZER_PINO, HIGH); // Ativar o buzzer
          vTaskDelay(pdMS_TO_TICKS(150));
          digitalWrite(BUZZER_PINO, LOW); // Desativar o buzzer
        }

        if (!objetoDetetadoAnterior && mensagemObjetoDetetado) { //evitar mensagens em duplicado
          mensagemObjetoDetetado = false;
        }

        SerialBT.println(valores);
      }

      if (buzzerAtivo && !ultimoEstadoBuzzer) {
        SerialBT.println("Acidente");
        ultimoEstadoBuzzer = true; // Atualizar o estado do Buzzer 

      } else if (!buzzerAtivo && ultimoEstadoBuzzer) {
        digitalWrite(BUZZER_PINO, LOW); // Parar o sinal do buzzer
        ultimoEstadoBuzzer = false; // Atualizar o estado anterior
      }

      if (SerialBT.available()) {
        String mensagem = SerialBT.readString();
        mensagem.trim();
        if (mensagem == "stop") {
          if (permitirAcidente) {
            acidenteAtivo = false;
            buzzerAtivo = false;
            digitalWrite(BUZZER_PINO, LOW);
            atualizaDetecaoAcidente();
            vTaskResume(tarefaHCSR04Handle);
            vTaskResume(tarefaDHT11Handle);
            permitirAcidente = true; // Permitir novo acionamento do acidente
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(200));  // Atraso entre as mensagens
    }
  }
}