#!/usr/bin/env python
# -*- coding:Utf-8 -*
from math import *
import os
import serial
import time
import datetime
from hauteursoleil import *
import math
from GPSMMS import *
#      pour obtenir un shutdown précédant la coupure d'alimentation
# sur gestion de la charge de batterie et de l'heure signalant envoyé
# par un GPIO en entrée sur raspberry et recevant le signal du wipy
# tentative pour arrêter système à parir de python à tester

from subprocess import call

###############  Variables globales
listmeswi=["température","boussolesolaire","boussolerobot","inclinaison"]

#///////////////  Arret propre avant coupure alimentation par Wipy //////////////////////////////
def shutdown():
  call("sudo nohup shutdown -h now", shell=True)

#////////////////////////////////////////////////////////////////////
# identification de l'énumération des ports USB
#pour identifier celui qui communique avec le WIPY
# on profite du système pour lister les ports ttyUSB* ttyS* ttyAM*
# dans fichier pour l'exploiter
sergpstexto= ""          # ce sont les variable qui contiendront les objets liaison
serarduino= ""
serwipy= ""
portGPSTEXTO=["USB", "cp210x", "GPSTEXTO", sergpstexto] # tableau initalisé
portARDUINO=["USB", "ch341", "ARDUINO", serarduino]     # avec en [0] type de liaison cablé et identifié par Os.Sytem
portWIPY=["S", "3f215040", "WIPY", serwipy]             # en [1] identifiant reconnu par Os.system

os.system ("dmesg | grep tty* > /home/pi/listtty.txt") # commande système d'inspection des liaisons

def rechercheport(port):                 # validation par fichier issu de os.system
  f= open("/home/pi/listtty.txt","r")
  while 1:
    ligne= f.readline()
    if ligne=="":
      break
    pos1=ligne.find(port[1])
    if pos1>=0:
      pos2=ligne.find(port[0],pos1)
      port[0]= ligne[pos2: pos2+(len(port[0])+1)]
      print (port[0])
      break
  f.close()
  if port[0]=="": # périphérique non reconnu
    print ("echec de la connection "+ port[2])
  else:
    portnom='/dev/tty'+ port[0]
    port[0]=portnom
    print(portnom)
    port[3] = serial.Serial(port=portnom,baudrate=115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=0.01)
    print(type(port[3]))
  return port

portGPSTEXTO = rechercheport(portGPSTEXTO) # validation du port
sergpstexto= portGPSTEXTO[3]   # pointer l' objet de connection
portARDUINO = rechercheport(portARDUINO)
serarduino= portARDUINO[3]
portWIPY = rechercheport(portWIPY)
serwipy= portWIPY[3]
print(sergpstexto)
print(serarduino)
print(serwipy)

#///////////////////////////////////////////////////////////////////////
# intialisation des GPIO de raspberry
import RPi.GPIO as GPIO  # bibliothèque pour gestion GPIO (attention au réservation série de nanpy)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) # les index seront ceux du connecteur raspberry

# ////////////////////activation de liaison  rasi<>arduino //////////////////
from nanpy import ArduinoApi
from nanpy import SerialManager
print(portARDUINO[0])
connection=SerialManager(device=portARDUINO[0]) # serial  connection to Arduino
arduino = ArduinoApi(connection=connection) # création de l'objet nanpy
print("debut3")

#------------  Intialisation de la carte  GPS TEXTO  ---------------

#at+cpin=1234
#at+csca="+33609001390"  # attention il s'agit du numéor du serveur SMS du réseau ici SFR
#at+cmgf=1
#at+cmgs="+33625620932"
# "bonjour tout le monde\r\n"
#b"\x1a\r\n"
#at+cmgr=0   #1 2 3 ... (rang du message)
#AT+CMGD=?
#AT+CMGD= 1,4  #index, delflag (0,1,2,3,4)  4 tous les messages, attention il faut 5 seconde pour message 10 pour 10
#AT+CMGL=?     liste les message dans la mémoire et donc exploitables
#AT+CMGL="ALL"   pour le voir
"""  module de gestion du module électronique GPS +  TEXTO   par les commande AT intégré au SIM 868
il a besoin d'un objet serial qui devra être ouvert par le __main__
il rend deux tableaux
    + listmessage une liste qui comporte les derniers messages reçus.
c'est le dernier élément qui donne le dernier en date, les deux premiers éléments sont toujours vide
    + listinfo qui renvoit la suite des données (trame du GPS  heure, coordonné, vitesse etc  voir la doc)
l'intialisation se fait selon les données inscrites en dur dans le tableau    tamponcommande
ce tableau contient en particulier: code pin, numero de l'opérateur pour SMS, et numéro du "confident"
    + ce module gère aussi la mise en marche. En détectant la réponse sur la liaison série
    il sait reconnaitre que la carte est active ou non et donc répète le reset au besoin
    en effet lors du demarrage du Raspberry la valeur de la GPIO 4 peut ne pas être
    celle attendue par défaut.
   """
try:
    initialisationGPSTEXTO(sergpstexto) # à remettre dans version travail
    point=donne_point(sergpstexto)
    envoi_texto(sergpstexto,point)
    listemessage=reception_texto(sergpstexto)# à la suite ce sera le traitement des messages puis leur effacement
    # ici traitement des messages
    suppression_messages(sergpstexto)  #à mettre dans la version travail

except KeyboardInterrupt:
    if sergpstexto != None:
        sergpstexto.write(b"AT+CGNSPWR=0\r\n")
        sergpstexto.write(b"AT+CGNSTST=0\r\n")
        sergpstexto.close()



print("debut1")

# Helper function to take HHMM.SS, Hemisphere and make it decimal:
def degrees_to_decimal(data, hemisphere):
    try:
        decimalPointPosition = data.index('.')
        degrees = float(data[:decimalPointPosition-2])
        minutes = float(data[decimalPointPosition-2:])/60
        output = degrees + minutes
        if hemisphere is 'N' or hemisphere is 'E':
            return output
        if hemisphere is 'S' or hemisphere is 'W':
            return -output
    except:
        return ""
# //////////////////////////////////////////////
gx = gy = 1
grayscale = blur = canny = False
#  initalisation des variables pour communicaiton série raspi/wipy
mesraspi=""
meswi=""

ordono="5"   # le code pour arret moteur total
boucle=0
b=0
i=0
reponse=0
reponse1=0
reponse2=0
reponse3=0
distance=30  # pour supposer que pas contact avec obstacle
distance1=0
distance2=0
impulsion=0.0 #"durée d'une fente codeuse pour calcul vitesse moteur
T1=0.0 #"temsp du debut de l'impulsion"
collapsus=0
daùro=0

print("suite")
# Moteurs droits
pinDD8 = 8 # marche avant
arduino.pinMode(pinDD8, arduino.OUTPUT)
pinDD6 = 6  # marche arrière
arduino.pinMode(pinDD6, arduino.OUTPUT)        # remplacer pinDD6 par pindDP6 si exploitation PWM

# Moteurs gauches
pinDD7 = 7 # marche avant
arduino.pinMode(pinDD7, arduino.OUTPUT)
pinDD9 = 9 # marche arrière
arduino.pinMode(pinDD9, arduino.OUTPUT)

# Moteurs azimut
pinDD2 = 2 # marche avant
arduino.pinMode(pinDD2, arduino.OUTPUT)
pinDD3 = 3 # marche arrière
arduino.pinMode(pinDD3, arduino.OUTPUT)

# Moteurs élévation
pinDD4 = 4 # marche avant
arduino.pinMode(pinDD4, arduino.OUTPUT)
pinDD5 = 5 # marche arrière
arduino.pinMode(pinDD5, arduino.OUTPUT)

# interrupteur de sécurité
pinDD10 = 10  # point bas du panneau
arduino.pinMode(pinDD10, arduino.INPUT_PULLUP)
pinDD11 = 11  # point haut du panneau
arduino.pinMode(pinDD11, arduino.INPUT_PULLUP)

M1,M2,M3,M4,IR,US=0,1,2,3,4,5
val=list(range(6))
capteur=0
label_text=""
print("suite1")


##   # IMPORTANT ! ignorer les premières
##   #lectures j usqu' à ce que Ax retourne
##   # quelque chose de valide
##   while pinA0. read( ) is None:
##
##      pass
##   while pinA1. read( ) is None:
##      pass
##   while pinA2. read( ) is None:
##      pass
##   while pinA3. read( ) is None:
##      pass
##
##   while pinA4. read( ) is None:
##      pass
##   while pinA5. read( ) is None:
##      pass


#         routine d'itinéraire /////////////////////////
def itineraire():
  global b,ordono
  if distance<20 :    # arretturgence
    print("arret d'urgence")
    ordono = "5"
   # repeat()  #  l'arret est l'oscasion de capture le paysage à find'analyse "
    # il faudra decider que faire après cet arret
  else:
    ############ il faut savoir ou on est donc acuisition des capteurs
    if val[IR]>0.2:
       arret()

#**************************************************************************
def communication():
  global ordono,daùro, capture
  print("ordono =", ordono)
  if ordono=="1":
     marche_av(daùro)
  elif ordono=="2":
     marche_ar(daùro)
  elif ordono=="3":
     virage_d(daùro)
  elif ordono=="4":
     virage_g(daùro)
  elif ordono=="5":
     arret()
  elif ordono== "6":
     solaire_bas(daùro)
  elif ordono== "7":
     solaire_haut(daùro)
  elif ordono=="8":
     solaire_est(daùro)
  elif ordono=="9":
     solaire_ouest(daùro)
  ordono="0"

#************************************************************************
def marche_av (daùro):
   arret()
   t= time.time()
   daùro=daùro+t
   while (time.time()< daùro):  # pas mini 2
     arduino.digitalWrite(pinDD8, 1)
     arduino.digitalWrite(pinDD6,0)
     arduino.digitalWrite(pinDD7, 1)
     arduino.digitalWrite(pinDD9,0)
def marche_ar (daùro):
   arret()
   t= time.time()
   daùro=daùro+t
   while (time.time()< daùro):  # pas mini 2
     arduino.digitalWrite(pinDD8, 0)
     arduino.digitalWrite(pinDD6,1)
     arduino.digitalWrite(pinDD7, 0)
     arduino.digitalWrite(pinDD9,1)
def virage_d (daùro):
   arret()
   t= time.time()
   daùro=daùro+t
   while (time.time()< daùro):  # pas mini 2
     arduino.digitalWrite(pinDD8, 1)
     arduino.digitalWrite(pinDD6,0)
     arduino.digitalWrite(pinDD7, 0)
     arduino.digitalWrite(pinDD9,1)
def virage_g(daùro):
   arret()
   t= time.time()
   daùro=daùro+t
   while (time.time()< daùro): # pas mini 2
     arduino.digitalWrite(pinDD8, 0)
     arduino.digitalWrite(pinDD6,1)
     arduino.digitalWrite(pinDD7, 1)
     arduino.digitalWrite(pinDD9,0)
def arret():
   arduino.digitalWrite(pinDD8,0)
   arduino.digitalWrite(pinDD6,0)
   arduino.digitalWrite(pinDD7,0)
   arduino.digitalWrite(pinDD9,0)

   arduino.digitalWrite(pinDD2,0)
   arduino.digitalWrite(pinDD3,0)
   arduino.digitalWrite(pinDD4,0)
   arduino.digitalWrite(pinDD5,0)

# repeat()   # futur inspection du paysage qui ne marchepas
######################" solaire
def solaire_bas (daùro):
  arret()
  t= time.time()
  daùro=daùro+t
  #print (t)
  while (time.time()< daùro): # pas mini 1
      if (arduino.digitalRead(10)==True):
         arduino.digitalWrite(pinDD4,1)
         arduino.digitalWrite(pinDD5,0)
      else:
         break
  arret()
  time.sleep(3)  # pause pour stablisation avant mesure .....

def solaire_haut (daùro):
  arret()
  t= time.time()
  daùro=daùro+t
  #print (t)
  while (time.time()< daùro):  # pas mini 1
    if (arduino.digitalRead(11)==True):
        arduino.digitalWrite(pinDD4,0)
        arduino.digitalWrite(pinDD5,1)
    else:
        break
  arret()
  time.sleep(3)

def solaire_est(daùro):
  arret()
  t= time.time()
  print ("solest",daùro)
  daùro=daùro+t
  #daùro=t+0.2

  while (time.time()< daùro):  # pas mini 0.2
      if (arduino.digitalRead(10)==True):
         arduino.digitalWrite(pinDD2,1)
         arduino.digitalWrite(pinDD3,0)
      else:
         #print ("arret")
         break
  arret()
  time.sleep(2)

def solaire_ouest(daùro):
  arret()
  t= time.time()
  print ("solouset",daùro)
  daùro=daùro+t
  #daùro=t+0.2
  while (time.time()< daùro): # pas mini 0.2
      if (arduino.digitalRead(10)==True):
         arduino.digitalWrite(pinDD2,0)
         arduino.digitalWrite(pinDD3,1)
      else:
         break
  arret()
  time.sleep(2)
#
#
##################################################################"


################################## mesure des larguer d'impulsion
# effacer de cette verison

#**********************************************************************

##
##   #--------------------------------------------------------------------
##   def valeurs_analogiques():
##     global capteur, label_text
##     if capteur==IR:
##      val[IR]=pinA4.read()
##      label_text = (str(val[IR]*1000))
##      #self.label.config( text = label_text)
##     elif capteur==US:
##      val[US]=pinA5.read()
##      label_text = (str(pinA5.read( )*1000))
##      #self.label.config( text = label_text)
##     elif capteur==M1:
##      val[M1]=pinA0.read()
##      label_text = (str(pinA0.read( )*1000))
##      #self.label. config( text = label_text)
##     elif capteur==M2:
##      val[M2]=pinA1.read()
##      label_text = (str(pinA1.read( )*1000))
##      #self.label.config( text = label_text)
##     elif capteur==M3:
##      val[M3]=pinA2.read()
##      label_text = (str(pinA2.read( )*1000))
##      #self.label.config( text = label_text)
##     elif capteur==M4:
##      val[M4]=pinA3.read()
##      label_text = (str(pinA3.read( )*1000))
##   #************************************************************** """



def commandemoteurs(ordono):
##    global commmande, capture
##    commande=int(ord(str(atemp)))
##    print(("Envoi de la valeur commande ",commande))
##    print(commande, "je suis passé par commandemoteur")
    print ("atemp",ordono)
    communication(ordono)
#--------------------------------------------------------------------

def saisie():
     global ordono
     ordono= input()
     print("ordono",type(ordono),"longuer", len(ordono), "valeur", ordono)

#----------------------------------------------
def  change():
    global a,capteur,collapsus

#//////////////////////////////////////////////
    itineraire()     # routine de pilotage
#//  routine de service

#  gestion de la communicaiton série avec le wipy
# meswi: variable chaine de ce qui sera envoyé vers le Wipy
# mesraspi: variable contenant ce qui est reçu venant du wipy 
def controlemeswi(meswi,listmeswi):
    """_summary_

    Args:
        meswi (chaine): variable chaine de ce qui sera envoyé vers le Wipy
        listmeswi (liste de chaines): variable globale contenant le vocabulaire du wipy

    Returns:
        variables binaire: de confiramation que le vocabulare est respecté
    """  
    if meswi in listmeswi :
        controle=1
    else:
        controle=0
    return controle    
def comwipy(meswi,mesraspi):
  if (serwipy.isOpen()):
        controle=controlemeswi(meswi,listmeswi)  
        if controle==1 :  # donc si la requête fait partie de listmeswi
            print("dans comwipy " ,  meswi)
            meswi=meswi+"\r\n"
            meswi=bytes(meswi,"utf-8")
            # envoie la requête du raspi vers wipy
            serwipy.write(meswi)
            meswi = ""
        if serwipy.inWaiting() != 0 :
            mesraspi = serwipy.readline().decode("utf-8","ignore") #on lit la ligne
            if mesraspi !="\r\n":
                print ("DANS COMWUPI   meswi= ",meswi," mesraspi = ",mesraspi)
  else:
        print ("Le serwipy n'a pas pu être ouvert !")

  return meswi,mesraspi
#---------------------------------------------------------
def traitemessage(mesraspi):  # traite les messages arrivant du wipy
    if mesraspi !="":
      mesraspi=mesraspi.strip()
#    if mesraspi != "\r\n":  
#      print(mesraspi,"++++") #çç
      if (mesraspi=="extinction éphéméride")or(mesraspi=="extinction décharge"):
        shutdown() # arret propre avant coupur alimentation anoncé par wipy
    mesraspi=""
     #--------------------------------------------------------------

def initvariableswipy():
  meswi,mesraspi=comwipy("misejourlevercoucher","")
  meswi,mesraspi=comwipy((str(tabheure[0])),"") # heure de lever "vrai" cf élévation >8°
  meswi,mesraspi=comwipy((str(tabminute[0])),"") # minute lever
  meswi,mesraspi=comwipy((str(tabheure[len(tabheure)-1])),"") # heure coucher
  meswi,mesraspi=comwipy((str(tabminute[len(tabminute)-1])),"") # minute coucher
  #print("misejourlevercoucher1")
  # date MMJJhhmmAA
  dh= datetime.datetime.now() # c'est l'heure du raspi qui doit être issu de l'initalisation du module GPS
  print("date raspi",str(dh))
  b=str(dh)
  dateheure=[b[:4],b[5:7],b[8:10],b[11:13],b[14:16],b[17:19]]
  meswi,mesraspi=comwipy("misejourdateheure","")
  for i in range(6):     # transmission année,mois,jour,heure,minute,seconde
    meswi,mesraspi=comwipy(dateheure[i],"")
  # calcul du tableau des instants en minutes de suivi de point solaire
  point=[]
  for i in range(len(tabheure)-1):
      point.append((tabheure[i]*60+tabminute[i]))
  return point

def reglagesolaire(): # gestion du panneau photovoltaique
    print("je passe par reglage solaire")
    global ordono,boucle,daùro
    mesureinclinterval=[0,0,0]
    mesureincli=0
    anglemaginterval=[0,0,0]
    anglemag=0
    date=datetime.datetime.now()
    heure=date.hour
    minute=date.minute
    print(heure,minute)
    for i in range(len(tabheure)-1): # recherche si c'est l'heure d'un pointage
        #print("i",i,(heure*60)+minute,">=",tabpointage[0],"and",(heure*60)+minute,"<",tabpointage[i])

        if (((heure*60)+minute >=tabpointage[0]) and  ((heure*60)+minute<tabpointage[i])):
            # réglage élévation
            elevation=tabelev[i] # extraction de l'élévation du pointage
            print("elevation",elevation)
            inclinaison=90-elevation # angle du panneau par rapport à l'horizontale
            if inclinaison >73:
                inclinaison= 73 # limite mécanique et inutile de faire mieux !
            contredescente=False
            contremontee=False   # drapeaux anti-oscillation
            mesureinclinterval,mesureincli=incli(meswi,mesraspi)
            while (inclinaison in mesureinclinterval)==False:
                mesureinclinterval,mesureincli=incli(meswi,mesraspi)
                if ((inclinaison > mesureincli) and (contremontee==False)):
                    contredescente=True
                    if contredescente==True:

                        if boucle > 10:
                           break
                    if abs(inclinaison-mesureincli)>=10:
                        daùro=(abs(inclinaison-mesureincli)/5)*0.5
                    else:
                        daùro=1
                    print(inclinaison,mesureincli)
                    print("durée élevation", daùro)
                    ordono="7"
                    communication()
                    boucle=boucle+1
                elif((inclinaison < mesureincli) and contredescente==False) :
                    contremontee=True
                    if contremontee==True:

                        if boucle > 10:
                           break
                    if abs(inclinaison-mesureincli)>=10:
                        daùro=(abs(inclinaison-mesureincli)/5)*0.5
                    else:
                        daùro=1
                    print(inclinaison,mesureincli)
                    print("durée élevation", daùro)
                    ordono="6"
                    communication()
                    boucle=boucle+1
                else:
                    break
            #réglage azimut, il faudra mettre en Variable globale: impulsion de base, et différents seuils
            azimut=tabazi[i] #extraction de l'azimut du pointage
            contreE=False
            contreW=False   # drapeaux anti-oscillation
            boucle=0
            anglemaginterval,anglemag=magnetometresolaire()
            while  (azimut in anglemaginterval)==False :
                anglemaginterval,anglemag=magnetometresolaire()
                print("azimut",azimut,anglemag)
                if ((azimut > anglemag+1) and (contreW==False)):
                    contreE=True
                    if contreE==True:
                        boucle=boucle+1
                        if boucle > 1:
                           break
                    if abs(azimut-anglemag)>=10:
                        daùro=((abs(azimut-anglemag))/5)*0.2
                    else:
                        daùro=0.2
                    print ("daùro",daùro)
                    ordono="9"
                    communication()
                elif((azimut < anglemag-1) and contreE==False) :
                    contreW=True
                    if contremontee==True:
                        boucle=boucle+1
                        if boucle > 1:
                           break
                    if abs(azimut-anglemag)>=10:
                        daùro=((abs(azimut-anglemag))/5)*0.2
                    else:
                        daùro=0.2
                    ordono="8"
                    communication()
                else:
                    break
            break
#/////////////////////////////////////////////////////////////////////////
def incli(meswi,mesraspi) :
    meswi="inclinaison"
    meswi,mesraspi = comwipy(meswi,mesraspi) 
    meswi=""
    while mesraspi == "" :
      meswi,mesraspi = comwipy(meswi,mesraspi)
    return meswi,mesraspi

def boussolesolaire(meswi,mesraspi) :
    meswi="boussolesolaire"
    meswi,mesraspi = comwipy(meswi,mesraspi) 
    meswi=""
    while mesraspi == "" :
      meswi,mesraspi = comwipy(meswi,mesraspi)
    print (mesraspi)
    return meswi,mesraspi
  
def boussolerobot(meswi,mesraspi) :
    meswi="boussolerobot"
    meswi,mesraspi = comwipy(meswi,mesraspi) 
    meswi=""
    while mesraspi == "" :
      meswi,mesraspi = comwipy(meswi,mesraspi)
    return meswi,mesraspi

def température(meswi,mesraspi) :
    meswi="température"
    meswi,mesraspi = comwipy(meswi,mesraspi) 
    meswi=""
    while mesraspi == "" :
      meswi,mesraspi = comwipy(meswi,mesraspi)
    print (mesraspi)
    return meswi,mesraspi
  
  
tabheure,tabminute,tabelev,tabazi=tablesuivi()
print (tabheure,"     ",tabminute,"     ",tabelev,"      ",tabazi) #çç  intialisation des tableaux journaliers
# en effet le programme est rebooter tous les jours par le wipy qui déclenche
# la mise sous tension et son arret selon heure je jour, élévation supérieure à 8 °
tabpointage=initvariableswipy() # réajuste selon tabheure et tabminute
print ("je rentre dans la boucle")
meswi="raspiOK" # pour signaler au wipy que raspi est prêt
pasprogramme=0

#exemple de programme général

while 1:
  meswi,mesraspi = comwipy(meswi,mesraspi)# regard sur la com uart avec wipy  
  if pasprogramme== 4:
      print("température")
      meswi,mesraspi=température(meswi,mesraspi)
                 
  if pasprogramme== 5:
      meswi,mesraspi=boussolerobot(meswi,mesraspi)
            
  if pasprogramme== 6:
      meswi,mesraspi=boussolesolaire(meswi,mesraspi)
      
  if pasprogramme== 7:
      meswi,mesraspi=incli(meswi,mesraspi)
      
#
  #meswi,mesraspi = comwipy(meswi,mesraspi)# regard sur la com uart avec wipy
#
  if mesraspi !="":
      #print("retour mesraspi ",mesraspi)
      traitemessage(mesraspi)
    #meswi=""# meswipy est le message adressé au wipy il est donc remis à vide une fois envoyé
  mesraspi=""
  #saisie ()  # commande manuelle
  ordono="0" # pour les tests valeur  "arrêt"
  #communication() # exécution des ordres moteur en passant par arduino (nanpy)  
  reglagesolaire()
  #boucle=0

  #change() # routine d'action automatique
  #shutdown() # ici pour tester la routine de shutdown
  pasprogramme=pasprogramme+1
  
print("tot")
##    print("erreur", sys.exc_info()[0])
##    sys.exit(0)
