#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de commande en tension des moteurs du robot T-Quad,
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.1.1 - 15/12/2016
##################################################################################

# Importe les fonctions Arduino pour Python
from pyduino_pcduino import *

# Imports pour la communication i2c avec l'Arduino Mega
from mega import Mega
mega = Mega()

import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

Nmoy = 1

omegaArriereDroit = 0.
codeurArriereDroitDeltaPos = 0
codeurArriereDroitDeltaPosPrec = 0

omegaArriereGauche = 0.
codeurArriereGaucheDeltaPos = 0
codeurArriereGaucheDeltaPosPrec = 0

omegaAvantDroit = 0.
codeurAvantDroitDeltaPos = 0
codeurAvantDroitDeltaPosPrec = 0

omegaAvantGauche = 0.
codeurAvantGaucheDeltaPos = 0
codeurAvantGaucheDeltaPosPrec = 0

# Variables nécessaires à la commande des moteurs
# Consignes de tension
vref = 0.
vrefArriereDroit = 0.
vrefArriereGauche = 0.
vrefAvantDroit = 0.
vrefAvantGauche = 0.

# Tension effectivement appliquée
commandeArriereDroit = 0.
commandeArriereGauche = 0.
commandeAvantDroit = 0.
commandeAvantGauche = 0.

# Saturations
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur

# Déclaration des variables pour la réception des données
typeSignal = 0
offset = 0.
amplitude = 0.
frequence = 0.
moteur1 = 0
moteur2 = 0
moteur3 = 0
moteur4 = 0

# Timeout de réception des données
timeout = 2
timeLastReceived = 0
timedOut = False

T0 = time.time()
dt = 0.01
i = 0
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

idecimLectureTension = 0
decimLectureTension = 6000
decimErreurLectureTension = 100

# Mesure de la tension de la batterie
# On la contraint à être supérieure à 7V, pour éviter une division par
# zéro en cas de problème quelconque
lectureTensionOK = False
tensionAlim = 7.4
while not lectureTensionOK:
    try:
        tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
        lectureTensionOK = True
    except:
        print("Erreur lecture tension")


#--- setup --- 
def setup():
    CommandeMoteurs(0, 0, 0, 0)
    
    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i, T0
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --

def CalculVitesse():
    global omegaArriereDroit, omegaArriereGauche, omegaAvantDroit, omegaAvantGauche, timeLastReceived, timeout, timedOut, \
        tdebut, codeurArriereDroitDeltaPos, codeurArriereGaucheDeltaPos, codeurAvantDroitDeltaPos, codeurAvantGaucheDeltaPos, \
        commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche, \
        vrefArriereDroit, vrefArriereGauche, vrefAvantDroit, vrefAvantGauche, vref, \
        codeurArriereDroitDeltaPosPrec, codeurArriereGaucheDeltaPosPrec, codeurAvantDroitDeltaPosPrec, codeurAvantGaucheDeltaPosPrec, \
        idecimLectureTension, decimLectureTension, decimErreurLectureTension, tensionAlim, \
        typeSignal, offset, amplitude, frequence, moteur1, moteur2, moteur3, moteur4
    
    tdebut = time.time()
        
    # Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
    try:        
        codeursDeltaPos = mega.read_codeursDeltaPos()
        codeurArriereDroitDeltaPos = codeursDeltaPos[0]
        codeurArriereGaucheDeltaPos = codeursDeltaPos[1]
        codeurAvantDroitDeltaPos = codeursDeltaPos[2]
        codeurAvantGaucheDeltaPos = codeursDeltaPos[3]
        
        # Suppression de mesures aberrantes
        if (abs(codeurArriereDroitDeltaPos - codeurArriereDroitDeltaPosPrec) > 20) or (abs(codeurArriereGaucheDeltaPos - codeurArriereGaucheDeltaPosPrec) > 20) or (abs(codeurAvantDroitDeltaPos - codeurAvantDroitDeltaPosPrec) > 20) or (abs(codeurAvantGaucheDeltaPos - codeurAvantGaucheDeltaPosPrec) > 20):
            codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
            codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec
            codeurAvantDroitDeltaPos = codeurAvantDroitDeltaPosPrec
            codeurAvantGaucheDeltaPos = codeurAvantGaucheDeltaPosPrec

        codeurArriereDroitDeltaPosPrec = codeurArriereDroitDeltaPos
        codeurArriereGaucheDeltaPosPrec = codeurArriereGaucheDeltaPos
        codeurAvantDroitDeltaPosPrec = codeurAvantDroitDeltaPos
        codeurAvantGaucheDeltaPosPrec = codeurAvantGaucheDeltaPos
    except:
        #print "Error getting data"
        codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
        codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec
        codeurAvantDroitDeltaPos = codeurAvantDroitDeltaPosPrec
        codeurAvantGaucheDeltaPos = codeurAvantGaucheDeltaPosPrec

    omegaArriereDroit = -2 * ((2 * 3.141592 * codeurArriereDroitDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaArriereGauche = 2 * ((2 * 3.141592 * codeurArriereGaucheDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaAvantDroit = -2 * ((2 * 3.141592 * codeurAvantDroitDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaAvantGauche = 2 * ((2 * 3.141592 * codeurAvantGaucheDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    
    tcourant = time.time() - T0
    # Calcul de la consigne en fonction des données reçues sur la liaison série
    if typeSignal == 0: # signal carré
        if frequence > 0:
            if (tcourant - (float(int(tcourant*frequence)))/frequence < 1/(2*frequence)):
                vref = offset + amplitude
            else:
                vref = offset
        else:
            vref = offset + amplitude
    else: # sinus
        if frequence > 0:
            vref = offset + amplitude * sin(2*3.141592*frequence*tcourant)
        else:
            vref = offset + amplitude

    # Application de la consigne sur chaque moteur
    vrefArriereDroit = moteur1 * vref
    vrefArriereGauche = moteur2 * vref
    vrefAvantDroit = moteur3 * vref
    vrefAvantGauche = moteur4 * vref

    # Calcul de la commande avant saturation
    # Si on n'a pas reçu de données depuis un certain temps, celles-ci sont annulées
    if (time.time()-timeLastReceived) > timeout and not timedOut:
        timedOut = True
        vrefArriereDroit = 0
        vrefArriereGauche = 0
        vrefAvantDroit = 0
        vrefAvantGauche = 0
        
    commande_avant_sat_ArriereDroit = vrefArriereDroit
    commande_avant_sat_ArriereGauche = vrefArriereGauche
    commande_avant_sat_AvantDroit = vrefAvantDroit
    commande_avant_sat_AvantGauche = vrefAvantGauche

    # Application de la saturation sur la commande
    if (commande_avant_sat_ArriereDroit > umax):
        commandeArriereDroit = umax
    elif (commande_avant_sat_ArriereDroit < umin):
        commandeArriereDroit = umin
    else:
        commandeArriereDroit = commande_avant_sat_ArriereDroit
    
    if (commande_avant_sat_ArriereGauche > umax) :
        commandeArriereGauche = umax
    elif (commande_avant_sat_ArriereGauche < umin):
        commandeArriereGauche = umin
    else:
        commandeArriereGauche = commande_avant_sat_ArriereGauche

    if (commande_avant_sat_AvantDroit > umax):
        commandeAvantDroit = umax
    elif (commande_avant_sat_AvantDroit < umin):
        commandeAvantDroit = umin
    else:
        commandeAvantDroit = commande_avant_sat_AvantDroit
    
    if (commande_avant_sat_AvantGauche > umax) :
        commandeAvantGauche = umax
    elif (commande_avant_sat_AvantGauche < umin):
        commandeAvantGauche = umin
    else:
        commandeAvantGauche = commande_avant_sat_AvantGauche

    CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche)

    # Lecture de la tension d'alimentation
    if idecimLectureTension >= decimLectureTension:
        try:
            tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
            idecimLectureTension = 0
        except:
            # On recommence la lecture dans decimErreurLectureTension * dt
            idecimLectureTension = idecimLectureTension - decimErreurLectureTension
            #print("Erreur lecture tension dans Loop")
    else:
        idecimLectureTension = idecimLectureTension + 1

        
    #print time.time() - tdebut
        
        
def CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    global tensionAlim
    
    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tensionArriereDroit = commandeArriereDroit
    tensionArriereGauche = commandeArriereGauche
    tensionAvantDroit = commandeAvantDroit
    tensionAvantGauche = commandeAvantGauche

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int_ArriereDroit = int(255 * tensionArriereDroit / tensionAlim)
    tension_int_ArriereGauche = int(255 * tensionArriereGauche / tensionAlim)
    tension_int_AvantDroit = int(255 * tensionAvantDroit / tensionAlim)
    tension_int_AvantGauche = int(255 * tensionAvantGauche / tensionAlim)

    # Saturation par sécurité
    if (tension_int_ArriereDroit > 255):
        tension_int_ArriereDroit = 255

    if (tension_int_ArriereDroit < -255):
        tension_int_ArriereDroit = -255

    if (tension_int_ArriereGauche > 255):
        tension_int_ArriereGauche = 255

    if (tension_int_ArriereGauche < -255):
        tension_int_ArriereGauche = -255

    if (tension_int_AvantDroit > 255):
        tension_int_AvantDroit = 255

    if (tension_int_AvantDroit < -255):
        tension_int_AvantDroit = -255

    if (tension_int_AvantGauche > 255):
        tension_int_AvantGauche = 255

    if (tension_int_AvantGauche < -255):
        tension_int_AvantGauche = -255

    # Commande PWM
    try:
        mega.moteursArriere(-tension_int_ArriereDroit, tension_int_ArriereGauche)
        mega.moteursAvant(-tension_int_AvantDroit, tension_int_AvantGauche)
        mega.moteursCRC(-tension_int_ArriereDroit + tension_int_ArriereGauche, -tension_int_AvantDroit + tension_int_AvantGauche)
    except:
        pass
        #print "Erreur moteurs"

    
def emitData():
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    delay(5000)
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 20)
        self.callback.start()
    

    def on_message(self, message):
        global vref, vrefArriereDroit, vrefArriereGauche, vrefAvantDroit, vrefAvantGauche, typeSignal, offset, amplitude, frequence, \
        moteur1, moteur2, moteur3, moteur4, timeLastReceived, timedOut
        
        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        timedOut = False;
        
        if jsonMessage.get('typeSignal') != None:
            typeSignal = int(jsonMessage.get('typeSignal'))
        if jsonMessage.get('offset') != None:
            offset = float(jsonMessage.get('offset'))
        if jsonMessage.get('amplitude') != None:
            amplitude = float(jsonMessage.get('amplitude'))
        if jsonMessage.get('frequence') != None:
            frequence = float(jsonMessage.get('frequence'))
        if jsonMessage.get('moteur1') != None:
            moteur1 = float(jsonMessage.get('moteur1'))
        if jsonMessage.get('moteur2') != None:
            moteur2 = float(jsonMessage.get('moteur2'))
        if jsonMessage.get('moteur3') != None:
            moteur3 = float(jsonMessage.get('moteur3'))
        if jsonMessage.get('moteur4') != None:
            moteur4 = float(jsonMessage.get('moteur4'))
        
            
        if not socketOK:
            typeSignal = 0
            offset = 0.
            amplitude = 0.
            frequence = 0.


    def on_close(self):
        global socketOK, vrefArriereDroit, vrefArriereGauche, vrefAvantDroit, vrefAvantGauche
        print 'connection closed...'
        socketOK = False
        vrefArriereDroit = 0.
        vrefArriereGauche = 0.
        vrefAvantDroit = 0.
        vrefAvantGauche = 0.

    def sendToSocket(self):
        global omegaArriereDroit, omegaArriereGauche, omegaAvantDroit, omegaAvantGauche, vref, \
        socketOK, commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), \
                                'Consigne':("%.2f" % vref), \
                                'omegaArriereDroit':("%.2f" % omegaArriereDroit), \
                                'omegaArriereGauche':("%.2f" % omegaArriereGauche), \
                                'omegaAvantDroit':("%.2f" % omegaAvantDroit), \
                                'omegaAvantGauche':("%.2f" % omegaAvantGauche), \
                                'Raw':("%.2f" % tcourant) \
                                + "," + ("%.2f" % vref) \
                                + "," + ("%.2f" % omegaArriereDroit) \
                                + "," + ("%.2f" % omegaArriereGauche) \
                                + "," + ("%.2f" % omegaAvantDroit) \
                                + "," + ("%.2f" % omegaAvantGauche) \
                                })
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    global vrefArriereDroit, vrefArriereGauche, vrefAvantDroit, vrefAvantGauche, device
    print 'Sortie du programme'
    vrefArriereDroit = 0.
    vrefArriereGauche = 0.
    vrefAvantDroit = 0.
    vrefAvantGauche = 0.
    CommandeMoteurs(0, 0, 0, 0)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 
    setup() # appelle la fonction setup
    print "Setup done."
    
    th = threading.Thread(None, emitData, None, (), {})
    th.daemon = True
    th.start()
    
    print "Starting Tornado."
    try:
        print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
    except:
        pass
        
    try:
        print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
    except:
        pass
    socketOK = False
    startTornado()


