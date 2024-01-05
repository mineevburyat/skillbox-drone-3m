#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Regislab
"""

from matplotlib import pyplot as plt


class VehicleSimpleDynamic:
    def __init__(self, mass, k_b, rotorCount, accInit, velInit, posInit):
        '''

        :param mass: масса аппарата
        :type mass: float
        :param k_b: коэффициент тяги двигателя
        :type k_b: float
        :param rotorCount: количество двигателей в системе
        :type rotorCount: int
        :param accInit: начальное значение ускорения ЛА
        :type accInit: float
        :param velInit: начальное значение скорости ЛА
        :type velInit: float
        :param posInit: начальное значение положения ЛА
        :type posInit: float

        '''
        self.mass = mass
        self.k_b = k_b
        self.rotorCount = rotorCount
        self.acceleration = accInit
        self.velocity = velInit
        self.position = posInit
        self.g = 9.81 

    def rightParts(self, rotorsAngularVel):
        '''

        :param rotorsAngularVel: угловая скорость двигателей
        :type rotorsAngularVel: float

        '''
        rotorsAngularVelSum = 0
        for i in range(self.rotorCount):
            rotorsAngularVelSum += rotorsAngularVel**2
        self.acceleration = (self.k_b * rotorsAngularVelSum) / self.mass - self.g

    def integrate(self, dt):
        '''

        :param dt: шаг моделирования
        :type dt: float

        '''
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt

    def calculatePosition(self, u, dt):
        '''

        :param u: управляющие воздействие
        :type u: float
        :param dt: шаг моделирования
        :type dt: float

        '''
        # Для определения положения вызываем метод для правых частей(то есть наших приращений от перемещения)
        # В данном случае приращением выступает ускорение нашей системы.
        # Для следования заданному целевому значению высоты передаем в метод наше управляющие воздействие u,
        # которое характеризует необходимую угловую скорость двигателей.
        self.rightParts(u)
        # Далее вызываем метод интегрирования
        # Интегрируем полученное при помощи функции правых частей ускорение и получаем скорость,
        # после интегрируем скорость и получаем положение.
        self.integrate(dt)

    def getPosition(self):
        '''

        :return: положение ЛА
        :rtype: float

        '''
        return self.position

    def getVelocity(self):
        '''

        :return: скорость ЛА
        :rtype: float

        '''
        return self.velocity

    def getAcceleration(self):
        '''

        :return: ускорение
        :rtype: float

        '''
        return self.acceleration


class ControlSystem():
    def __init__(self, k_p, k_i, k_d, controlLimit):
        '''

        :param k_p: коэффициент П регулятора
        :type k_p: float
        :param k_i: коэффициент И регулятора
        :type k_i: float
        :param k_d: коэффициент Д регулятора
        :type k_d: float
        :param controlLimit: ограничение по управляющему воздействию
        :type controlLimit: float

        '''
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.desiredPosition = 0
        self.error = 0
        self.errorPast = 0
        self.integral = 0
        self.controlLimit = controlLimit

    def setDesiredPosition(self, desiredPosition):
        '''

        :param desiredPosition: целевое положение ЛА
        :type desiredPosition: float

        '''
        self.desiredPosition = desiredPosition

    def PID(self, currentPosition, dt):
        '''

        :param currentPosition: текущее положение ЛА
        :type currentPosition: float
        :param dt: шаг моделирования
        :type dt: float

        '''
        self.error = self.desiredPosition - currentPosition
        self.integral += self.error * dt
        u = self.k_p * self.error + self.k_i * self.integral + \
            self.k_d * ((self.error - self.errorPast) / dt)
        self.errorPast = self.error
        u = self.saturation(u)
        return u

    def saturation(self, inputVal):
        '''

        :param inputVal: входное значение
        :type inputVal: float
        :return: выходное значение после прохождения проверки на ограничение
        :rtype: float

        '''
        if inputVal > self.controlLimit:
            inputVal = self.controlLimit
        elif inputVal < -self.controlLimit:
            inputVal = - self.controlLimit
        return inputVal


class Simulator():

    def __init__(self, Tend, dt, controlSys, dynamicModel):
        '''

        :param Tend: конечное время моделирования
        :type Tend: float
        :param dt: шаг моделирования
        :type dt: float
        :param controlSys: объект системы управления высотой ЛА
        :type controlSys: ControlSystem
        :param dynamicModel: объект модели ЛА
        :type dynamicModel: VehicleSimpleDynamic

        '''
        self.dt = dt
        self.Tend = Tend
        self.controlSys = controlSys
        self.dynamicModel = dynamicModel
        self.accList = []
        self.velList = []
        self.posList = []
        self.timeList = []

    def runSimulation(self):
        '''

       метод запускает моделирование системы от 0 до конечного времени Tend 
       с шагом dt

        '''
        time = 0
        while (time <= self.Tend):
            self.posList.append(self.dynamicModel.getPosition())
            self.velList.append(self.dynamicModel.getVelocity())
            self.accList.append(self.dynamicModel.getAcceleration())
            self.timeList.append(time)

            # рассчитываем новое управляющие воздействие
            u = self.controlSys.PID(self.dynamicModel.getPosition(), self.dt)
            self.dynamicModel.calculatePosition(u, self.dt)
            time += self.dt

    def showPlots(self):
        '''
        метод строит графики на основе измерений полученных в 
        ходе моделирования системы

        '''
        f = plt.figure(constrained_layout=True)
        
        gs = f.add_gridspec(3, 5)
        ax1 = f.add_subplot(gs[0, :-1])
        ax1.plot(self.timeList, self.posList)
        ax1.grid()
        ax1.set_title('position')

        ax2 = f.add_subplot(gs[1, :-1])
        ax2.plot(self.timeList, self.velList, "g")
        ax2.grid()
        ax2.set_title('velocity')

        ax3 = f.add_subplot(gs[2, :-1])
        ax3.plot(self.timeList, self.accList, "r")
        ax3.grid()
        ax3.set_title('acceleration')

        plt.show()


'''
 Объявим параметры для моделирования
'''
k_p = 100 #коэффициент Пропорционального регулирования
k_i = 10 #коэффициент Интегрального регулирования
k_d = 80 #коэффициент Дифференциального регулирования
dt = 0.01 # шаг моделирования системы (например одна сотая секунды)

Tend = 20 # конечное время моделирования (например 20 сек)

# Масса ЛА
mass = 0.006
# Коэффициент тяги двигателя ЛА
k_b = 3.9865e-08
# Количество двигателей ЛА
rotorCount = 4
# Ограничение на угловую скорость двигателей рад/сек
motorSpeedLimit = 3000

'''
Создадим объект контроллера и объект для нашей математической модели
'''
controller = ControlSystem(k_p, k_i, k_d, motorSpeedLimit)
uavSimpleDynamic = VehicleSimpleDynamic(mass, k_b, rotorCount, 0, 0, 0)
'''
Установим целевое положение для нашей системы
'''
controller.setDesiredPosition(20)


"""
Создадим объект симулятора и передадим в него контроллер
 и математическую модель
"""
sim = Simulator(Tend, dt, controller, uavSimpleDynamic)
sim.runSimulation()  # запуск симулятора
sim.showPlots()  # построение графиков
