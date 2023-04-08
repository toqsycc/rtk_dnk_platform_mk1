/*
*   Основной код управления платформы
*   08.04.2023 @toqsycc
*
*   Является обновлённой, исправленной и адаптированнй версией кода
*   платформы Wunderwagen совместного авторства @euthanasiaTrip и @toqsycc
*
*   Совместим с любыми UART-устройствами передачи данных
*
*   Перед компиляцией выставить по надобности   __DEBUG_MODE - режим отладки через ПК;
*                                               __DISCRETE_SPEED - дискретный режим установки скоростей;
*                                               __GEARMODE_SPEED - режим установки скоростей "коробкой передач"
*
*   Рекомендуется проверить самостоятельно углы отклонений сервоприводов
*
*   Скачать через Менеджер библиотек пакет 'Adafruit PWM Servo Driver'
*/
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --------- Распиновка платы --------- //
#define ENGINE_IN1_A                50  // Вывод IN1+A драйвера DBH-12
#define ENGINE_IN2_A                50  // IN2+A
#define ENGINE_PWM_A                50  // EN+A
                                        //
#define ENGINE_IN1_B                50  // IN1+B
#define ENGINE_IN2_B                50  // IN2+B
#define ENGINE_PWM_B                50  // EN+B
                                        //
#define SERVO_CLAW                  0   // Идентификатор сервопривода захвата
#define SERVO_TURN                  1   // Идентификатор сервопривода вращения
#define SERVO_BASE                  2   // Идентификатор сервопривода отклонения
                                        //
#define CAM_SWITCH                  50  // Вывод реле переключения камеры

// ----- Блок констант компиляции ----- //
#define SERVO_DRIVER_ADDRESS      0x40  // Адрес I2C драйвера сервоприводов PCA9685
#define SERVO_MANUAL_STEP            5  // Угол одного шага сервопривода в режиме ручного управления
#define WIRELESS_UART_PORT     Serial1  // Идентификатор UART-порта с Wi-Fi модулем
#define DEBUG_UART_PORT         Serial  // Идентификатор UART-порта для отладки через ПК
                                        //
#define CLAW_MAXVAL                270  // Максимальное значение угла захвата
#define CLAW_INITIAL                90  // Значение угла захвата по умолчанию
#define CLAW_MINVAL                  0  // Минимальное значение угла захвата
                                        //
#define TURN_MAXVAL                270  // Максимальное значение угла вращения
#define TURN_INITIAL                90  // Значение угла вращения по умолчанию
#define TURN_MINVAL                  0  // Минимальное значение угла вращения
                                        //
#define BASE_MAXVAL                270  // Максимальное значение угла отклонения
#define BASE_INITIAL                90  // Значение угла отклонения по умолчанию
#define BASE_MINVAL                  0  // Минимальное значение угла отклонения
                                        //
#define WIRELESS_BAUDRATE       115200  // Скорость передачи данных с Wi-Fi модулем
#define DEBUG_BAUDRATE            9600  // Скорость передачи данных с ПК при отладке
                                        //
#define FORWARD_LEFT        0x00, 0x01  // Конфигурация IN1+IN2 левого ряда для езды вперед
#define FORWARD_RIGHT       0x01, 0x00  // Конфигурация IN1+IN2 правого ряда для езды вперед
#define BACKWARD_LEFT     FORWARD_LEFT  // Конфигурация IN1+IN2 левого ряда для езды назад
#define BACKWARD_RIGHT   FORWARD_RIGHT  // Конфигурация IN1+IN2 правого ряда для езды назад
                                        //
#define __DEBUG_MODE                    // Включает режим отладки через ПК. Закомментить для релиза
#define __DISCRETE_SPEED                // Включает режим дискретной скорости (xG * 255/5).
                                        // Закомментить при использовании предопределенных скоростей
//#define __GEARMODE_SPEED                // Включает режим предопределенных скоростей (speed = gearBoxValues[xG])
                                        // Закомментить при использовании дискретных скоростей
                                        //
#define CAMERA_FRONT              0x00  // Значение для включения фронтальной камеры
#define CAMERA_BACK               0xFF  // Значение для включения камеры заднего вида
                                        //
#define CONTROL_CODE_CLAW_MIN       75  // Код контроля клешни, положение "полностью открыта"
#define CONTROL_CODE_CLAW_MAX      100  // Код контроля клешни, положение "полностью закрыта"
                                        //
#define CONTROL_CODE_SERVO_DEF      42  // Код восстановления положения манипулятора по умолчанию
                                        //
#define CONTROL_CODE_SERVO_INCR     43  // Код выборки сервопривода, инкремент индекса
#define CONTROL_CODE_SERVO_DECR     44  // Код выборки сервопривода, декремент индекса
                                        //
#define CONTROL_CODE_ANGLE_INCR     24  // Код увеличения угла сервопривода, инкремент
#define CONTROL_CODE_ANGLE_DECR     43  // Код увеличения угла сервопривода, декремент
                                        //
#define CONTROL_CODE_CAM_CHANGE     40  // Код смены текущей камеры
                                        //
#define CONTROL_CODE_MVMT_HALT       0  // Код остановки моторов
                                        //
#define CONTROL_CODE_MVMT_BACK1G     1  // Код установки движения назад. Первая передача
#define CONTROL_CODE_MVMT_BACK2G     2  // Вторая передача
#define CONTROL_CODE_MVMT_BACK3G     3  // ...
#define CONTROL_CODE_MVMT_BACK4G     4  // ...
#define CONTROL_CODE_MVMT_BACK5G     5  // Пятая передача 
                                        //
#define CONTROL_CODE_MVMT_FORW1G     6  // Код установки движения вперед. Первая передача
#define CONTROL_CODE_MVMT_FORW2G     7  //
#define CONTROL_CODE_MVMT_FORW3G     8  //
#define CONTROL_CODE_MVMT_FORW4G     9  //
#define CONTROL_CODE_MVMT_FORW5G    10  //
                                        //
#define CONTROL_CODE_MVMT_RGHT1G    11  // Код установки движения вправо. Первая передача
#define CONTROL_CODE_MVMT_RGHT2G    12  //
#define CONTROL_CODE_MVMT_RGHT3G    13  //
#define CONTROL_CODE_MVMT_RGHT4G    14  //
#define CONTROL_CODE_MVMT_RGHT5G    15  //
                                        //
#define CONTROL_CODE_MVMT_LEFT1G    16  // Код установки движения влево. Первая передача
#define CONTROL_CODE_MVMT_LEFT2G    17  //
#define CONTROL_CODE_MVMT_LEFT3G    18  //
#define CONTROL_CODE_MVMT_LEFT4G    19  //
#define CONTROL_CODE_MVMT_LEFT5G    20  //

// Встроенная защита от дурака на этапе компиляции
#if defined(__DISCRETE_SPEED) && defined(__GEARMODE_SPEED)
#error Разрешается использовать только один режим скоростей. Закомментируйте неиспользуемый.
#endif

// ------ Конфигурация gearMode ------- //
#ifdef __GEARMODE_SPEED                 //
#define GEARMODE_1G                 75  // Скорость первой передачи
#define GEARMODE_2G                120  // Скорость второй передачи
#define GEARMODE_3G                160  // ...
#define GEARMODE_4G                200  // ...
#define GEARMODE_5G                255  // Скорость пятой передачи
#endif

// ----- Класс управления мотором ----- //
class Engine 
{
private:
  uint8_t           _en1,               // IN1
                    _en2,               // IN2
                    _fdb,               // CT - нереализованный функционал обратной связи
                    _pwm;               // EN
  uint8_t           _en1v,              // Значение IN1 в оперативной памяти
                    _en2v,              // Значение IN2 в оперативной памяти
                    _spd;               // Ширина импульсов EN в текущий момент
public:
  /*
   *  Инициализация интерфейса мотора
   *
   *  Входные аргументы:
   *  e1 : Номер пина IN1
   *  e2 : Номер пина IN2
   *  pwm : Номер пина EN
   *
   *  Возвращаемое значение:
   *  -> &Engine
   */
  Engine(uint8_t e1, uint8_t e2, uint8_t pwm)
    : _en1(e1), _en2(e2), _pwm(pwm) 
  {
    pinMode(e1, OUTPUT);
    pinMode(e2, OUTPUT);
  }
  /*
   *  Деинициализация интерфейса мотора
   *
   *  Входные аргументы:
   *  нет
   *
   *  Возвращаемое значение:
   *  нет
   */
  ~Engine();
  /*
   *  Включение мотора в установленном ОЗУ направлении
   *  с заданной пользователем скоростью
   *
   *  Входные аргументы:
   *  spd : Ширина импульсов EN [0..255]
   *
   *  Возвращаемое значение:
   *  -> нет
   */
  inline void enable(uint8_t spd) 
  {
    digitalWrite(_en1, _en1v);
    digitalWrite(_en2, _en2v);
    analogWrite(_pwm, spd);
  }
  /*
   *  Отключение мотора
   *
   *  Входные аргументы:
   *  нет
   *
   *  Возвращаемое значение:
   *  -> нет
   */
  inline void disable() 
  {
    analogWrite(_pwm, 0x00);
  }
  /*
   *  Запись направления вращения мотора в ОЗУ
   *
   *  Входные аргументы:
   *  e1 : Полярность первого пина колодки [0..1]
   *  e2 : Полярность второго пина колодки [0..1]
   *
   *  Комментарии:
   *  ! Значения e1 и e2 должны различаться, иначе мотор не включится
   *
   *  Возвращаемое значение:
   *  -> нет
   */
  inline void set(uint8_t e1, uint8_t e2) 
  {
    _en1v = e1;
    _en2v = e2;
  }
};

// ----- Вспомогательные функции ------ //
// Инициализирует пин с реле переключения сигнала камер
inline void initCameraSwitch(uint8_t &buffer)
{
  buffer = CAMERA_FRONT;
  pinMode(CAM_SWITCH, OUTPUT);
  digitalWrite(CAM_SWITCH, buffer);
}
// Переключает состояние реле
inline void selectCamera(uint8_t buffer)
{
  digitalWrite(CAM_SWITCH, buffer);
}
// Ехать вперёд
inline void moveForward(Engine* &engineLeft, Engine* &engineRight)
{
  engineLeft->set(FORWARD_LEFT);
  engineRight->set(FORWARD_RIGHT);
}
// Ехать назад
inline void moveBackward(Engine* &engineLeft, Engine* &engineRight)
{
  engineLeft->set(BACKWARD_LEFT);
  engineRight->set(BACKWARD_RIGHT);
}
// Поворачивать налево
inline void moveLeft(Engine* &engineLeft, Engine* &engineRight)
{
  engineLeft->set(BACKWARD_LEFT);
  engineRight->set(FORWARD_RIGHT);
}
// Поворачивать направо
inline void moveRight(Engine* &engineLeft, Engine* &engineRight)
{
  engineLeft->set(FORWARD_LEFT);
  engineRight->set(BACKWARD_RIGHT);
}
// Включить моторы
inline void moveGo(Engine* &engineLeft, Engine* &engineRight, uint8_t speed)
{
  engineLeft->enable(speed);
  engineRight->enable(speed);
}
// Остановить моторы
inline void moveHalt(Engine* &engineLeft, Engine* &engineRight)
{
  engineLeft->disable();
  engineRight->disable();
}
// Обновить положение осей манипулятора
inline void updateManipulatorState(Adafruit_PWMServoDriver* &driver, uint16_t* buffer)
{
  driver->setPWM(SERVO_CLAW, 0x00, buffer[SERVO_CLAW]);
  driver->setPWM(SERVO_TURN, 0x00, buffer[SERVO_TURN]);
  driver->setPWM(SERVO_BASE, 0x00, buffer[SERVO_BASE]);
}

// ------ Глобальные переменные и объекты ------- //
uint8_t                     currentCameraSource;  // Буфер для хранения текущего источника сигнала
uint16_t                    servoAngleBuffer[] =  // Буфер для хранения текущих углов сервоприводов
{
  CLAW_INITIAL, TURN_INITIAL, BASE_INITIAL
};
constexpr uint16_t          servoMinValues[] =    // Константа для хранения минимальных углов сервоприводов
{
  CLAW_MINVAL, TURN_MINVAL, BASE_MINVAL
};
constexpr uint16_t          servoDefValues[] =    // Константа для хранения углов сервоприводов по умолчанию
{
  CLAW_INITIAL, TURN_INITIAL, BASE_INITIAL
};
constexpr uint16_t          servoMaxValues[] =    // Константа для хранения максимальных углов сервоприводов
{
  CLAW_MAXVAL, TURN_MAXVAL, BASE_MAXVAL
};
#ifdef __GEARMODE_SPEED
constexpr uint8_t           gearBoxValues[] =     // Константа для хранения скоростей передачи при использовании gearMode
{
  GEARMODE_1G, GEARMODE_2G, GEARMODE_3G, 
  GEARMODE_4G, GEARMODE_5G  
};
#endif

Engine                      *motorsLeft,          // Указатель на объект интерфейса управления левым рядом моторов
                            *motorsRight;         // Указатель на объект интерфейса управления правым рядом моторов
Adafruit_PWMServoDriver     *servoDriver;         // Указатель на объект интерфейса управления сервоприводами

void setup() 
{
  // Шаг 1. Инициализация портов GPIO
  motorsLeft = new Engine                         // Инициализация интерфейса управления левым рядом моторов
    (
      ENGINE_IN1_A,
      ENGINE_IN2_A, 
      ENGINE_PWM_A
    );
  motorsRight = new Engine                        // Инициализация интерфейса управления правым рядом моторов
    (
      ENGINE_IN1_B, 
      ENGINE_IN2_B, 
      ENGINE_PWM_B
    );
  initCameraSwitch(currentCameraSource);          // Инициализация пина управления источником сигнала камер

  // Шаг 2. Инициализация устройств связи
  WIRELESS_UART_PORT.begin(WIRELESS_BAUDRATE);    // Инициализация порта с Wi-Fi модулем
  WIRELESS_UART_PORT.setTimeout(15);              // Установка времени таймаута связи
#ifdef __DEBUG_MODE                               // Компилировать в случае если объявлен флаг __DEBUG_MODE
  DEBUG_UART_PORT.begin(DEBUG_BAUDRATE);          // Инициализация порта для отладки через ПК
  DEBUG_UART_PORT.println("Step 2 OK");
#endif

  // Шаг 3. Инициализация драйвера сервоприводов
  servoDriver = new Adafruit_PWMServoDriver
  (
    SERVO_DRIVER_ADDRESS
  );
  servoDriver->setPWMFreq(60);                    // Установка частоты импульсов в 60 Гц
#ifdef __DEBUG_MODE  
  DEBUG_UART_PORT.println("Step 3 OK");
#endif
  delay(1500);

  // Шаг 4. Перевод сервоприводов в положение по умолчанию
  updateManipulatorState(servoDriver, servoAngleBuffer);

  // Шаг 5. Выдержка времени на установку манипулятора в положение по умолчанию
  // Отладка: отписаться, что устройство завершило инициализацию.
  delay(5000);
#ifdef __DEBUG_MODE
  DEBUG_UART_PORT.println("Machine READY");
#endif
}

void loop() 
{
  static long       wirelessData;                 // Локальный буфер для хранения данных из потока
  static uint8_t    selectedServo(SERVO_CLAW);    // Локальный буфер для хранения выбранного сервопривода
  static uint8_t    currentSpeed(0);              // Локальный буфер для хранения текущей скорости

  if (WIRELESS_UART_PORT.available())             // Получить данные из потока если они доступны
  {
    wirelessData = WIRELESS_UART_PORT.parseInt();

#ifdef __DEBUG_MODE
    DEBUG_UART_PORT.print("Received sequence: "); // Отписка о полученных данных
    DEBUG_UART_PORT.write(wirelessData);          //
    DEBUG_UART_PORT.println(";");                 //
#endif

    // Коды диапазона CONTROL_CODE_CLAW:
    // Отвечают за сжатие клешни
    // Обновление положения происходит в конце цикла обработки данных
    if (wirelessData >= CONTROL_CODE_CLAW_MIN && wirelessData < CONTROL_CODE_CLAW_MAX)
      servoAngleBuffer[SERVO_CLAW] = map          // Ремап диапазона значений CONTROL_CODE к CLAW
      (
        wirelessData,                             // Текущее значение
        CONTROL_CODE_CLAW_MIN,                    // Нижняя граница исходного диапазона
        CONTROL_CODE_CLAW_MAX,                    // Верхняя граница исходного диапазона
        CLAW_MINVAL,                              // Нижняя граница результирующего диапазона
        CLAW_MAXVAL                               // Верхня граница результирующего диапазона
      );
    
    // Коды диапазона CONTROL_CODE_SERVO
    // Отвечают за выбор текущей оси манипулятора для последующего управления
    else if (wirelessData == CONTROL_CODE_SERVO_INCR && selectedServo < SERVO_BASE)
      selectedServo++;
    else if (wirelessData == CONTROL_CODE_SERVO_DECR && selectedServo > SERVO_CLAW)
      selectedServo--;

    // Коды диапазона CONTROL_CODE_ANGLE
    // Отвечают за изменение угла выбранной оси на предопределенный шаг
    else if (wirelessData == CONTROL_CODE_ANGLE_INCR && servoAngleBuffer[selectedServo] < servoMaxValues[selectedServo])
      servoAngleBuffer[selectedServo] += SERVO_MANUAL_STEP;
    else if (wirelessData == CONTROL_CODE_ANGLE_DECR && servoAngleBuffer[selectedServo] > servoMinValues[selectedServo])
      servoAngleBuffer[selectedServo] -= SERVO_MANUAL_STEP;

    // Код CONTROL_CODE_SERVO_DEF
    // Отвечает за восстановление положения манипулятора по умолчанию
    else if (wirelessData == CONTROL_CODE_SERVO_DEF)
      memcpy(servoAngleBuffer, servoDefValues, sizeof servoDefValues);

    // Код CONTROL_CODE_CAM_CHANGE
    // Осуществляет смену текущего источника видеосигнала путём инверсии байта
    else if (wirelessData == CONTROL_CODE_CAM_CHANGE)
      currentCameraSource = ~currentCameraSource;

    // Коды группы CONTROL_CODE_MVMT
    // Осуществляют установку направления и скорости движения
    else if (wirelessData >= CONTROL_CODE_MVMT_HALT && wirelessData <= CONTROL_CODE_MVMT_LEFT5G)
    {
      switch (wirelessData)
      {
      // Остановка машины
      case CONTROL_CODE_MVMT_HALT:
        currentSpeed = 0;
        break;

      // Секция блоков езды назад
      case CONTROL_CODE_MVMT_BACK1G:
        moveBackward(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[0];
#else
        currentSpeed = 255 / 5;
#endif
        break;
      case CONTROL_CODE_MVMT_BACK2G:
        moveBackward(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[1];
#else
        currentSpeed = (255 / 5) * 2;
#endif
        break;
      case CONTROL_CODE_MVMT_BACK3G:
        moveBackward(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[2];
#else
        currentSpeed = (255 / 5) * 3;
#endif
        break;
      case CONTROL_CODE_MVMT_BACK4G:
        moveBackward(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[3];
#else
        currentSpeed = (255 / 5) * 4;
#endif
        break;
      case CONTROL_CODE_MVMT_BACK5G:
        moveBackward(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[4];
#else
        currentSpeed = 255;
#endif
        break;

      // Секция блоков езды вперед
      case CONTROL_CODE_MVMT_FORW1G:
        moveForward(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[0];
#else
        currentSpeed = 255 / 5;
#endif
        break;
      case CONTROL_CODE_MVMT_FORW2G:
        moveForward(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[1];
#else
        currentSpeed = (255 / 5) * 2;
#endif
        break;
      case CONTROL_CODE_MVMT_FORW3G:
        moveForward(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[2];
#else
        currentSpeed = (255 / 5) * 3;
#endif
        break;
      case CONTROL_CODE_MVMT_FORW4G:
        moveForward(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[3];
#else
        currentSpeed = (255 / 5) * 4;
#endif
        break;
      case CONTROL_CODE_MVMT_FORW5G:
        moveForward(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[4];
#else
        currentSpeed = 255;
#endif
        break;

      // Cекция блоков поворота направо
      case CONTROL_CODE_MVMT_RGHT1G:
        moveRight(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[0];
#else
        currentSpeed = 255 / 5;
#endif
        break;
      case CONTROL_CODE_MVMT_RGHT2G:
        moveRight(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[1];
#else
        currentSpeed = (255 / 5) * 2;
#endif
        break;
      case CONTROL_CODE_MVMT_RGHT3G:
        moveRight(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[2];
#else
        currentSpeed = (255 / 5) * 3;
#endif
        break;
      case CONTROL_CODE_MVMT_RGHT4G:
        moveRight(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[3];
#else
        currentSpeed = (255 / 5) * 4;
#endif
        break;
      case CONTROL_CODE_MVMT_RGHT5G:
        moveRight(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[4];
#else
        currentSpeed = 255;
#endif
        break;

      // Cекция блоков поворота налево
      case CONTROL_CODE_MVMT_LEFT1G:
        moveLeft(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[0];
#else
        currentSpeed = 255 / 5;
#endif
        break;
      case CONTROL_CODE_MVMT_LEFT2G:
        moveLeft(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[1];
#else
        currentSpeed = (255 / 5) * 2;
#endif
        break;
      case CONTROL_CODE_MVMT_LEFT3G:
        moveLeft(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[2];
#else
        currentSpeed = (255 / 5) * 3;
#endif
        break;
      case CONTROL_CODE_MVMT_LEFT4G:
        moveLeft(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[3];
#else
        currentSpeed = (255 / 5) * 4;
#endif
        break;
      case CONTROL_CODE_MVMT_LEFT5G:
        moveLeft(motorsLeft, motorsRight);
#ifdef __GEARMODE_SPEED
        currentSpeed = gearBoxValues[4];
#else
        currentSpeed = 255;
#endif
        break;
      }
    }

    // Конец итерации обработки данных:
    // Обновление положения сервоприводов, запуск моторов, смена камер...
    updateManipulatorState(servoDriver, servoAngleBuffer);
    moveGo(motorsLeft, motorsRight, currentSpeed);
    selectCamera(currentCameraSource);
  }
  else
  {
    if (currentSpeed) 
      moveHalt(motorsLeft, motorsRight);          // В противном случае (таймаут) отключаем моторы
#ifdef __DEBUG_MODE
    DEBUG_UART_PORT.println("Timeout or no data. Idle mode, engines halted!");
#endif
  }

  // Задержка на разрешение выполнения функций
  delay(7);
}
