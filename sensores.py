import numpy as np
import math as m


class Calibracion:
    def __init__(self,setpoints:list)->list:
        self.setpoints = setpoints
        self.tiempoInit=5 #tiempo en segundos desde que se inicia la calibración para garantizar que el sensor se estabilice
        self.offsetValido=0.05 #permito una variación del 5% como oscilación válida en la medición.
        self.tiempoMedido=10 #tiempo de medición para cada setpoint en segundos para tomar una medida como válida y estabilizada
        self.m=0 #pendiente de la recta de calibración
        self.b=0 #ordenada al origen de la recta de calibración
        self.parametrosCalib=[] #lista con los parámetros de calibración (m,b)
        
class Medicion:
    def __init__(self,*args, **kwargs):
        self.medicion=0
        self.numMediciones=1 #numero de emdiciones para promediar
        if "mediciones" in kwargs:  #si se especifica el número de mediciones para promediar
            self.numMediciones=kwargs["mediciones"]
    
class Sensor:
    def __init__(self,name:str) -> None:
        self.name=name
        calibracion=Calibracion()
        medicion=Medicion()
    
