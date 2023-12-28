## Libsetila

  Libsetila е C++ библиотека за комуникација со I2C/SPI уреди кај SOC/SBC со Linux оперативен систем без употреба на  драјвери за целните I2C/SPI уреди. Библиотеката обезбедува класи за опис на SPI и ISP уреди и методи за размена на податоци со нив. Главна цел на библиотеката е да го олесни пишувањето на софтвер за комуникација со разни I2C/SPI уреди од т.н. кориснички простор (во ориг. user space) за коишто не постојат соодветни драјвери.

  За прием/испраќање на податоци кон/и од уредите на I2C/SPI податочната магистрала, библотеката користи стандардни влезно/излезни функциски повици кон системските драјвери за I2C/SPI управувачи. Драјверите за овие управувачи мора да бидат вчитани пред користење на библиотеката. Библиотеката не нуди поддршка за обратка на прекини (орг. interrupts) генерирани од страна на I2C/SPI уредите. 

Во библиотеката се вклучени:
- класи за управувачи со I2C/SPI податочни магистрали (во ориг. masters);
- класи за претставување на I2C/SPI уреди (во ориг. I2C/SPI slaves);
- образоци (во ориг. templates) за основни контејнери за складирање на податоци (анг. Basic Data Containers (BDC)): пласт (stack), FIFO редици и кружен бафер. Овие податочни структури може да се искористат за складирање на податоците исчитани од уредите/сензорите;
- дигитални филтри за филтрирање на исчитувањата од сензорите од несакани пречки и шумови: тековно на располагање е Moving Average Window филтер;
- поддршка за некои популарни и распространети I2C/SPI уреди (види дел за поддржан хардвер).
 
Библиотеката не е наменета за истовермено користење во неколку апликациски нишки (не е thread safe). 

### Софтверски побарувања

За превод на библиотеката потребен е C++ компајлер кој го поддржува C++11 стандардот.

### Хардвер

#### Поддржани хардверски платфроми и оперативен систем

Библиотеката би требало да работи на секоја SOC/SBC платформа со вградени I2C/SPI управувачи (во орг. I2C/SPI masters), која поддржува извршување на Linux оперативен систем со соодветни драјвери за I2C/SPI контролери и C++11 преведувач.

Библотеката е успешно теститрана на следниве системи:
- Raspberry Pi Compute Module 4 Revision 1.1. со оперативен систем: Debian 12 ("bookworm");
- Raspberry Pi 2, 3 model B and Pi Zero/W со оперативен систем: Rasbian GNU/Linux version 10 ("buster");
- Beagle Bone Black revision A со Debian GNU/Linux 9 ("stretch").

И покрај тоа што библиотеката претставува општа алатка за полесно пишување на код за комуникација со I2C/SPI базирани уреди, истата вклучува готови класи (види папка „devices“) за неколку популарни периферни уреди, како на пример: сензори за притисок, влажност и температура, аналогно-дигитален конвертор, ултразвучен сонар итн.. Овие уреди служеа за тестирање на кодот на библиотеката при нејзинито развој и доказ за неговата функционалност. 

#### I2C уреди
- ST L3GD20 three axes digital output gyroscope (FIFO mode and single measurement);
- ST HTS221 humidity and temperature sensor: single acquisition(ONE SHOT measurement), FIFO_MEAN mode, adjustable output data rate and resolution;
- ST LPS25H pressure and temperature sensor: single acquisition(ONE SHOT measurement), adjustable output data rate and resolution;
- ST LPS22HB pressure and temperature sensor: single acquisition(ONE SHOT measurement);
- Microchip MCP9808 temperature sensor: temperature reading, no support for alarms/interrupts;
- Analog Devices ADT7410 температурен сензор: мерење на температура, без поддршка за аларми/прекини;
- Bosh BMP085 сензор за притисок и температура. Целосно поддржан;
- Bosh BMP180 сензор за притисок и температура. Целосно поддржан;
- SRF02 ултразвучен сонар.

#### SPI уреди

- Microchip MCP3204 2.7V, 4-Channel, 12-Bit Analog/Digital Converter

### Инсталација

Упатство за инсталација на библиотеката е содржано во документот INSTALL.

### Документација

Детална документација за библиотеката се наоѓа во папката "doc".

### Примена и практични примери

  Составен дел од библиотеката се и примери за нејзината практична употреба, сместени во папката „examples“. Библиотеката мора да биде инсталирана на целната платформа пред превод на изворниот код на примерите. Како примери за користење на класите од библиотеката за програмирање на комуникација со нови I2C/SPI уреди кои не се дирекно поддржани од страна на библиотеката, може да послужи кодот од папката „devices“.

### Test Hardware

Функционалноста на библиотеката е тестирана со користење на следниве уреди:

- Adafruit BMP085 модул (Adafruit product ID 391);
- Adafruit 10-DOF IMU Breakout (Adafruit product ID 1604);
- Adafruit BMP180 модул (Adafruit product ID 1603);
- Adafruit MCP9808 High Accuracy I2C Temperature Sensor Breakout Board (Adafruit product ID 1782); 
- Adafruit ADT7410 High Accuracy I2C Temperature Sensor Breakout Board (Adafruit product ID 4089);
- [MCP3204 тест модул](https://github.com/positronic57/libmcp3204/tree/master/example/hardware) 
- SRF02 - I2C/Serial ултра-звучен сонар
- Arduino MKR ENV Shield rev2
- Pi Sense HAT;
- Raspberry Pi модел B rev1;
- Raspberry Pi 2 модел B;
- Raspberry Pi 3 модел B;
- Raspberry Pi Zero/W;
- Raspberry Pi Compute Module 4 Revision 1.1;
- Beagle Bone Black revision A.

**ПРЕДУПРЕДУВАЊЕ:**
Изворниот код е достапен каков што е, без никаква гаранција за неговата функционалност и содржина. Користењето на кодот е на сопствен ризик!
Авторот на кодот не презема никава одговорност за штети кои можеби би биле предизвикани со користење на овој код.

**ОГРАДУВАЊЕ:** 
Изворниот кодот е резултат на хоби дејност и авторот не е поврзан ниту пак е во партнерство со некој од прозводителите на уреди/електронски компоненти/софтверски алатки спомнати во кодот, документацијата или описот на овој проект. Авторот не искажува никаква поврзаност со, претензија или сопственост врз трговските имиња наведени во проектот. Сите трговски имиња и заштитни знаци се во сопсвеност на нивните иматели.