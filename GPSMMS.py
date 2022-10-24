#!/usr/bin/python
# -*- coding:Utf-8 -*-
#at+cpin=1234
#at+csca="+33609001390"  # attention il s'agit du numéor du serveur SMS du réseau ici SFR
#at+cmgf=1
#at+cmgs="+33625620932"   # mon portable
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
import RPi.GPIO as GPIO  # bibliothèque pour gestion GPIO (attention au réservation série de nanpy)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) # les index seront ceux du connecteur raspberry    
import serial
import time

def lectureserie(ser):
    data=""    
    while ser.inWaiting() > 0 : 
        data = ser.readline()  #ser.inWaiting()))
        #print(type(data)," données  ",data)
        break
    return data

def point_immediat(listinfo):
    pass
    return "heure"+listinfo[2]+"\r\n"+"latitude  "+listinfo[3]+"\r\n"+"longitude  "+listinfo[4]

def envoi_texto(ser,message):  
    ser.write(b"AT+CGNSTST=0\r\n") # arrêt du flux du GPS
    retour=videbuffer(ser)
    ser.write(b'AT+CMGS="+33625620932"\r\n')
    retour= videbuffer(ser)
    #ser.write(bytes(message,encoding="utf-8"))
    ser.write(bytes(message,encoding="utf-8"))
    ser.write(b"\x1a\r\n") # 0x1a : send   0x1b : Cancel send
    time.sleep(5) # delai pour achever la procédure d'envoi
    retour=videbuffer(ser)
    print(" retour de l'envoie de message   ",retour)
    
    
def lecturemessages(ser):
    ser.write(b'AT+CMGL="ALL"\r\n')
    time.sleep(3)
    retour = videbuffer(ser)
    return retour
   

def reception_texto(ser):
    data=""
    lmes=[]
    listmessage=[]  # liste globale qui rapporte les messages reçus
    ser.write(b"AT+CGNSTST=0\r\n") # pour arrêter le flux des données GPS
    purge=videbuffer(ser)
    print("purge  ",purge)
    data=lecturemessages(ser)    
    print(" suite des messages   ", data)
    print(type(data))
    b=data.decode("utf8","replace")
    print("Conversion    ",b)
    lmes=[]
    n=0
    try:
        while True:
            n=b.index('\n')
            c=b[:n]
            print("longueur  ",len(c),c)
            if len(c)==1 or "CMGL" in c:
                pass
            else:        
                lmes.append(b[:n])
                print("ligne isolée  ",b[:n])
            b=b[n+1:]
            #print("ligne restante",b)      
    except:
        pass
        
    for i in range(len(lmes)):
        print("message  :",i,"   ",lmes[i].rstrip())
    
    
    listmessage=lmes  # on élimine le premier +GMGL?
    print("liste transmise     ",  listmessage  )
    
    return listmessage

def initialisationGPSTEXTO(ser):
    resetSIM868(ser) 
    print("retour reset")
    num=0
    tamponcommande = ["AT+CSCA=\"+33609001390\"\r\n","AT+CMGF=1\r\n","AT+CGNSPWR=1\r\n","AT+CGNSINF\r\n"] # liste des ordres d'initalisation
    #tamponcommande = ["AT+CPIN=1234\r\n","AT+CSCA=\"++33609001390\"\r\n","AT+CMGF=1\r\n","AT+CGNSPWR=1\r\n","AT+CGNSINF\r\n"] # liste des ordres d'initalisation
    # ATTENTION le numéro qui suit at+csca est celui du serveur sms dur réseau ici +33609001390 pour le réseau SFR
    nbparam=len(tamponcommande)
    while num < nbparam : # mettre
        ser.write(bytes(tamponcommande[num],encoding="utf-8"))
        retour=videbuffer(ser)
        print (retour)
        num=num+1
        print(num)
    return

def videbuffer(ser) :
    time.sleep(0.1)
    retour = ser.read(ser.inWaiting())
    #print (retour)
    return retour

def supprimermessages(ser):
    ser.write(b"AT+CMGD=1,4\r\n")
    time.sleep(2)
    retour=videbuffer(ser)
    print(retour)

def donne_point(ser):
    data=""
    point=""
    listinfo=[]   # liste globale qui rapporte les données du GPS
    T0=time.time()
    
    while point=="" :
        ser.write(b"AT+CGNSINF\r\n")
        time.sleep(0.1)
        #data=lectureserie(ser) # comme d'habitude c'est l'accusé réception qui répète l'ordre AT
        #print("retour d'ordre",data)
        #time.sleep(0.5)
        data=videbuffer(ser)
        print(data)
        data=str(data)
        listinfo=data.split(",")        # histoire de retirer le "\r\n"
        if len(listinfo)==2:              # extraction de la gangue
            listinfo=str(listinfo[1])     # converion en chaine de caractère
            listinfo=listinfo.split(',')  # contruction du tableau grace au séparateur ","
        if len(listinfo)> 4 :
            #print("drapeau  ", listinfo[2])
            if listinfo[1]=="1": # signifie que les donnée GPS sont  valides
                point=point_immediat(listinfo)
                print(point,"//////////////")
                #time.sleep(1.0)
            else:
                point=""
                if time.time()-T0 > 10: # après test mettre 30
                    point="satellites perdus"
                    break                   
        else:
            if time.time()-T0 > 12: #après test mettre 60
                point="dysfonctionnement GPS et plus"
                break
        #print(".....................................................")
    videbuffer(ser)
    print("+++  ", point)
    #time.sleep(3)
    return point

def resetSIM868(ser): # indispensable pour initialiser la liaison série
    p=4
    data=""
    GPIO.setup(p, GPIO.OUT)    
    GPIO.output(p,GPIO.HIGH)
    GPIO.output(p,GPIO.LOW)
    print("pin 5  low")
    time.sleep(0.5)
    GPIO.output(p,GPIO.HIGH)
    time.sleep(20)
    data = lectureserie(ser)  # le module répond toujours avec l'ordre env
    print("amorce   ",data)
    while data != b'OK\r\n':
        ser.write(b"AT\r\n")
        time.sleep(5)
        data = lectureserie(ser)  # le module répond toujours avec l'ordre env
        print(data)        
    data=""
 
    
if __name__ == "__main__"  :
    
    ser = serial.Serial("/dev/ttyUSB0",115200) # USB0 ou USB1 ... suivant configuration
    try:
        initialisationGPSTEXTO(ser) # à remettre dans version travail
        point=donne_point(ser)
        envoi_texto(ser,point)
        listemessage=reception_texto(ser)# à la suite ce sera le traitement des messages puis leur effacement
        # ici traitement des messages 
        supprimermessages(ser)  #à mettre dans la version travail
           
    except KeyboardInterrupt:
        if ser != None:
            ser.write(b"AT+CGNSPWR=0\r\n")
            ser.write(b"AT+CGNSTST=0\r\n")
            ser.close()


