#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#   les ignes  contenant " #ç  et ##ç " sont des lignes contenant des uart.write de test
# et est donc facile par recherche remplacement des le préfixer par "#" pour les inhiber
""" programme micropython pour Wipy 3  qui gère l'alimentation suivant l'état de la batterie
+ en fonction du potentiel en amont et en aval du relai P18 P17
+ en fonction des heures de lever et coucher de soleil
+ en fonction de la détection de contact sur P8 "intrusion"
+ en plein jour si la batterie atteint se limite le sommeil profond est déclenché pour une heure avant de reconsidérer
si une recherche a pu se produire. En pareil cas le redémarrage se fait pour  1/4 heure
+ Dans la situation de sommeil profond le réveil survient sur condition d'heure (réveil normal)
 ou sur critère d'intrusion (P8)
+ la liaison série avec le raspberry est dévolue à l'échange pour mise à l'heure sur donnée GPS
et les messages d'intrusion de nuit (reveil sur broche 8) et pour déclencher dans le bon ordre l'arret du rasberry avant coupe
d'alimentation
////////////////////////////////////////////////////////////////////////////////////
attention ce programme contient l'exploitation d'un multiplexeur I2C TCA9548A
par défaut on ne connecte à A0 A1 A2 à rien , la connection à 3,3volt de ces ligne d'adresse permet d'incrémenter l'adresse
du multiplexer  dont l'adresse de base sur le bus I2C est 0x70  ainsi avce ce troi fil d'adresse
il est possible d'obtenir la communication avec 8 multiplexeurs qui recoivent dont on programme les adresses ainsi de 0xF0 à 0xF7.
Ceci étant l'ouverture d'un des canaux du multiplexeur se fait par une "commande envoyé sur le bus I2C lui même.
par exemple pour ouvrir le canal "0" du multiplexeur il faut envoyer sur le bus I2c  par writeto(0xF0,1) ensuite on constate
par exemple en scanant le bus que c'est bien le composant qui répond sur ce canal et lui seul. un print du scan donne
 la ou les adresses des composants connectés sur ce port suivi toujours de l'adresse du multiplexeur lui même, ici 112  puisque
 les lignes d'adresses ne sont pas actives et que donc par defaut l'adresse I2c du multiplexeur est 0xF0  soit 112  valeur numérique.
 par la on comprend que les autres ports étant isolés nous pourrons mettre jsuqu'à 8 de composant identique de mêmes adresses
 sans générer de conflit.  par exemple un write(0xf0, 128) permettra de communiquer uniquement avec le composant connecté au port 8.
 la valeur tranmise dans cette ordre de connexion est fait un octet dont chaque bit active un port
 d'où les seules valeurs autorisées: 1,2,4,8,16,32,64,128.  Un scan après un commande type write(0xF0, 5)
 qui en valeur digitale donne 00000101 entre comme seule réponse invariable 112 qui indique que le mutiplexeur qui répond toujours
 présent à son adesse n'a ouvert aucun de ses ports

+ retour d'expérience sur la nécessité de gérer le reset. En effet en cas de reset du Master I2c ici le wipy,
les registres du multiplexeur garde en mémoire l'état du bus lors de la déconnection. Donc lors de la reinitialisation un plantage survient.
il faut donc générer un un reset en mettant au niveau bas la broche Reset du multiplexeur

"""
import machine
from machine import ADC, Pin, RTC,UART,I2C
import utime
import time
import math

#******** configure la liason série "1" du wipy pour communication avec raspberry
uart = UART(1,baudrate=115200,bits=8,parity=None,stop=1)
rtc = RTC()
uart.write("je me lance")
uart.write("\r\n")
print("c'est fait")
# configure le bus I2C du Wipy
bus = I2C(0,I2C.MASTER, baudrate=100000)
print(bus)

#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#  comprendre pour quelle raison le réveil se produit : normal ou durant le sommeil prévu -> intrusion
# il reste a exploiter ces informations
(raison1,broche) = machine.wake_reason()
trestant = machine.remaining_sleep_time()
uart.write(str(machine.remaining_sleep_time()))
uart.write("\r\n")
#uart.write ("interruption du sommeil: raison du réveil, broche, temps restant "+str(raison1)+" "+str(broche)+" "+str(trestant))
# raison1  = 0 pour premier demarrage,  1  en cas de réveil prématuré par intrusion,  2 pour un réveil à l'heure programmé
# broche est NONE sauf en cas de réveil programmé et montre alors la cause (PIN8) en cause pour le réveil
# trestant  vaut 0 sauf en cas de reveil prémature il indique alors le temps qui le sépare d'un réveil normal
uart.write("\r\n")
if raison1 !=0:
    #pycom.heartbeat(True)  # disable the heartbeat LED
    pass
raison2 = machine.reset_cause()
uart.write("raison du boot"+str(raison2))
uart.write("\r\n")

if trestant==0: # il s'agit d'un réveil normal
    #pycom.rgbled(0x007f00) # green
    direchose=""
    pass
else: #  il s'agit d'un réveil sur intrusion qu'il faudra gérer par message liaison série
    uart.write("intrusion")
    uart.write("\r\n")
    direchose="intrusion"
    # gestionintrusion()
time.sleep(10)
#//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
# explication ici paramétrage de la pin P8 pour déclencher le réveil du sleep
# dans la phase de sauvegarde batterie ou seul le wypi est alimenté et en mode de sommeil.
#cette ligne 8 sera lié à un conducteur ouvert faisant le pourtour du boitier robot+"  "+str( si bien
#quune main venant en contact amènera la sortie prématurée du mode veille  déclenchant
#la procédure antivol nocturne .... la procédure antivol diurne sera plutot déclenchée
# par anomalie des données GPS  .... le tout amenant à émission de texto
gachette = Pin('P8', mode=Pin.IN, pull=Pin.PULL_DOWN)
machine.pin_sleep_wakeup([gachette], mode=machine.WAKEUP_ANY_HIGH)  #, enable_pull=True)

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
# initialisation de la broche  17 et broche 18 en convertisseur A-D(objet "adc")
adc = ADC(0)
vbatterie = adc.channel(pin='P18') # mesure voltage batterie
vcircuit = adc.channel(pin='P17') # controle voltage en aval du relai bistable
# initialisation de la broche 11  pour reset logiciel du multiplexeure I2c
resetI2Cmultiplexeur = Pin('P11', mode=Pin.OUT)
resetI2Cmultiplexeur.value(0)
time.sleep(1)
resetI2Cmultiplexeur.value(1)

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#initialisation variable globale
uart.write("initalisation des variables globales+\r\n")
uart.write("\r\n")
coupealim=False
echantmesures = list(range(10))  # pour stocker l'échantillon des mesure dont en retiendra le mode
direchose = ""  # message à destination du raspi
alimflag = 0     # 0 pour alimentation coupée  1 pour alimentation en fonction
#voltbat = 0 # mode de l'échantillon des mesures(10) pour écarter les artéfacts
#voltcircuit = 0 # idem pou circuit
#tempssommeil=0
# tableau pour gestion date heure à transformer en tuple pour alimenter RTC
dateheure = [2020, 12, 7, 9, 30, 23, 0, None]
# valeur qu'il faudra mettre à jour selon ephéméride
coucherlever = [8, 30, 20, 30]
heurelever = 8
minutelever = 40
heurecoucher = 20
minutecoucher = 40
instantarret=0
instantmarche=0
uneheure= 60  #3600    # a porter à 3600 soit une heure de recharge si en plien jour la batterie est basse
unquartheure=60  # 900 # a porter à 900
rtc = RTC(datetime=(2020, 12, 7, 9, 30, 23, 0, None)) # intialisaiton avant donnée GPS

# charge de sécurité
charge = 3800   #valeur ADC pour 12 volt avec 1 volt au pont diviseur 500Koms/le pont diviseur 300 kOhms/100 Kohms
drapeaupanne = 0 # drapeau des pannes(1   c'est la panne alimentation)
instant=time.time()
# broche pour le controle du relai bistable
broche2 = Pin('P2', mode=Pin.OUT)
broche2.value(0) # broche pour impulsion sur le relai bistable
#couplée logiquement avec controle au alimflag
# le shutdown se fera par message sur la liaison série

t = rtc.now()
uart.write(str(t))
uart.write("\r\n")


#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#                        Fontions de controle alimentation

def controlebatterie():
    for x in range(10):
      mes = vbatterie()
      echantmesures[x] = mes
      #uart.write("tension", mes) #çç valeur 0-4095 1,8 volt en 12 bit
      #utime.sleep_ms(10)
    echantmesures.sort()
    #uart.write(echantmesures, "   liste des mesures") #çç
    voltbat = echantmesures[4]  # valeur mode(-1)
    #uart.write("voltbat=", voltbat) #ç
    return voltbat

def controlecircuit():
    global voltcircuit
    for x in range(10):
      mes = vcircuit()
      echantmesures[x] = mes
#      uart.write("tension"+"  "+str(mes)) #çç
#      uart.write("\r\n")
#      utime.sleep_ms(10)
    echantmesures.sort()
    #uart.write(echantmesures, "   liste des mesures") #çç
    voltcircuit = echantmesures[4]
    #uart.write("voltcircuit", voltcircuit) #ç
    return voltcircuit

def impulsion():
    broche2.value(1)
    utime.sleep_ms(100)
    broche2.value(0) # fin de l'impulsion de bascule d relai bistable

def coupurealimentation(direchose,coupealim,instantmarche,alimflag):
    if (time.time()-instantmarche > unquartheure) : # si la mise en marche remonte à moins de 1/2 heure on continue
        uart.write(str(time.time()-instantmarche)+"  "+str(unquartheure))
        uart.write("\r\n")
        while True:
            voltcircuit=controlecircuit()
            if voltcircuit> 1000:
                impulsion()
                time.sleep(1)
                voltcircuit = 500 # pour test
                # ici interviendra le gestion en bon ordre de l'arrêt du raspberry sauvegarde etc .....
                #un message sera envoyé sur UART cette fin
            if voltcircuit<1000 :
                uart.write("coupure alimentation")
                uart.write("\r\n")
                if direchose=="extincnuit" :
                    temps = rtc.now() # lecture de l'heure acuelle
                    tempssommeil = (((heurecoucher-temps[3])+(24-heurecoucher)+ heurelever)*3600)+(((minutecoucher-temps[4])+(minutelever-3))*60)*1000 # en ms
                    uart.write(str(tempssommeil))
                    uart.write("\r\n")
                if direchose=="extincfaible":
                # ici la routine de mise en sommeil (pas de sommeil profond plus complexe à traiter
                    tempssommeil= 60000  #  60 secondes  UNIQUEMENT PENDANT LES TESTS puis mise à 1 h
                # on laisse coupealim à True
                alimflag = 0
                uart.write("entrée en sommeil profond")
                uart.write("\r\n")
                sommeil(tempssommeil)
            utime.sleep_ms(100)
    else:
        coupealim=False # donc du fait des conditions de stabilité on remet à plus tard la coupure
        pass
    return coupealim,alimflag

def gestionalimentation(drapeaupanne,coupealim,alimflag,direchose,instantmarche,instantarret,charge, voltbat, voltcircuit):
    temps = rtc.now()
    #uart.write(t)   #ç heure t3[3] minute temps[4]
    #uart.write(heurelever, minutelever, temps[3], temps[4], heurecoucher, minutecoucher, voltbat) #ç
    #uart.write(str(voltbat)+"  "+str(voltcircuit))
    #uart.write("\r\n")
    if voltcircuit > charge-100: # définir l'état du relai, indispensable lors du boot
        alimflag=1
    else:
        alimflag=0
    if drapeaupanne != 1 : #si le relai n'a pas été détecté comme en panne
        if ((temps[3]*60+temps[4]) >= (heurelever*60+minutelever)) and ((temps[3]*60+temps[4]) <= (heurecoucher*60+minutecoucher)):
            jour=True
            #uart.write("fait jour")
            #uart.write("\r\n")
        else:
            jour=False
            uart.write("fait nuit")
            uart.write("\r\n")
        if((voltbat < charge) and (time.time()-instantmarche> unquartheure)): #and alimflag == 1):
            if drapeaupanne == 0:
                uart.write("extinction décharge")#ç
                uart.write("\r\n")
                direchose = "extincfaible"
                instantarret=time.time() # l'instant servira a empêcher les changement d'état par fluctuation mineure des mesures pendant une durée
                coupealim=True
            #instant=time.time()
        elif (jour==True and voltbat > charge+50 and (time.time()-instantarret > uneheure)):
            x = 0
            while voltcircuit < voltbat-500 : # donc le raspberry et l'arduino pas alimentés
                controlecircuit()
                if(voltcircuit >= charge-100): #  sauf erreur tolérance perte en charge
                    break
                else:
                    impulsion()
                    instantmarche=time.time()
                    uart.write("mise en route avec concordance relai ")#ç
                    uart.write("\r\n")
                    utime.sleep_ms(300)  #stabilisation de la valeur en entrée sur adc
                    x = x+1
                    if x > 3:
                        alimflag=0
                        drapeaupanne = 1
                        uart.write("panne relai alimentation")#ç
                        uart.write("\r\n")
                        break
            alimflag=1
        elif (jour==False):  # il fait nuit
            if drapeaupanne == 0:
                uart.write("extinction éphéméride")#ç
                uart.write("\r\n")
                direchose = "extincnuit"
                coupealim=True
    return drapeaupanne,coupealim,alimflag,direchose,instantmarche,instantarret




#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
# fonction de communicaiton serie entre wipy et  Raspi par uart

def recevoir(): # routine des données reçues du raspi
    global peto,heurelever, minutelever, heurecoucher, minutecoucher, rtc,direchose
    peto = acquisition()
    
    if peto != None :
        if peto == "misejourlevercoucher": 
            for i in range(4):
                coucherlever[i] = acquisition()
    #            uart.write(str(coucherlever[i])+"*****"+"  "+str( i))
    #            uart.write("\r\n")
            heurelever = int(coucherlever[0])
            minutelever = int(coucherlever[1])
            heurecoucher = int(coucherlever[2])
            minutecoucher = int(coucherlever[3])
        if peto == "misejourdateheure": #acquisition de la date système du raspberry venant par GPS
             # fabrication du tuple
            for i in range(6):
                dateheure[i] = int(acquisition())
    #            uart.write(str(dateheure[i])+ "*****"+"  "+str( i))
    #            uart.write("\r\n")
            dateheure[6] = 0
            dateheure[7] = None
            # tableau > tuple
            tempstuple = tuple(dateheure)
    #        uart.write(str(tempstuple))
    #        uart.write("\r\n")
            rtc = RTC(datetime=tempstuple) # mise à date et heure
#        if  "inclinaison" in peto :
#            print("PASSE ICI POUR INCLNAISON")
#            mesureincliinterval,mesureincli=incli(0) 
#            print("INCLINAISON", mesureincli)
#            direchose= "inclinaison"+ str(mesureincli)
        if "temperature" in peto:
            print("TEMPERATURE")
            direchose=interrogation_aht10(0)
            direchose=str(direchose)
            print("renvoi température  :", direchose)
        if "inclinaison" in peto:
            direchose=interrogation_adxl345(0)
            direchose=str(direchose)
            print("renvoie incli solaire  ;",direchose)
        if "boussolesolaire" in peto:
            direchose=interrogation_lsm303(0)
            direchose=str(direchose)
            print("renvoie boussole solaire   ;",direchose)
        if "boussolerobot" in peto:
            direchose=interrogation_lsm303(1)
            direchose=str(direchose)
            print("renvoie boussole robot   ;",direchose)
            
            
 

def acquisition(): #remise en forme chaine  
    lecture = uart.readline() # si readline attention potentiellment bloquant attente du retour à la ligne et pas de time out sur wpy
    #print("acquisition brute  ",lecture)
    if lecture != None:
        print("lecture brute   ", lecture)
        lecture=lecture.decode("utf-8","ignore")
        #lecture=str(lecture)
        print("acquisition du wipy",lecture)

    return lecture
 #
def emettre(): 
    global direchose    
    if direchose != "":
         dire = direchose + "\r\n"  # fin de ligne pour le tratiemeent dans raspi
         print("wypi envoi     ",dire)
         uart.write(dire)
         #uart.write("\r\n")
         direchose = ""           # remet la variable global à ""
 #
#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#  déclenchement et gestion du sommeil profond
def sommeil(tempssommeil):
    tempssommeil=60000 # temps de 60 seconde en ms uniquement pour les test
    machine.deepsleep(tempssommeil)   # sera remplacé par machine.deepsleep(tempssommeil)
#/////////////////////////////////////////////////////////////////////////////
# classe de l'objet ADXL345   accéléromètre 3 axes
class ADXL345 :
    EARTH_GRAVITY_MS2 = 9.80665
    SCALE_MULTIPLIER = 0.004
    DATA_FORMAT = 0x31
    BW_RATE = 0x2C
    POWER_CTL = 0x2D

    BW_RATE_1600HZ = 0x0F
    BW_RATE_800HZ = 0x0E
    BW_RATE_400HZ = 0x0D
    BW_RATE_200HZ = 0x0C
    BW_RATE_100HZ = 0x0B
    BW_RATE_50HZ = 0x0A
    BW_RATE_25HZ = 0x09

    RANGE_2G = 0x00
    RANGE_4G = 0x01
    RANGE_8G = 0x02
    RANGE_16G = 0x03

    MEASURE = 0x08
    SOMMEIL = 0x00
    AXES_DATA = 0x32
    address = None
    def __init__(self, address=0x53):
        self.address = address
        self.set_bandwidth_rate(self.BW_RATE_25HZ)
        time.sleep(0.5)
        self.set_range(self.RANGE_2G)
        time.sleep(0.5)
        #self.enable_measurement(self.MEASURE)

    def enable_measurement(self,valeur):
        bus.writeto_mem(self.address,self.POWER_CTL, valeur)  #self.MEASURE)

    def set_bandwidth_rate(self,rate_flag):
        bus.writeto_mem(self.address,self.BW_RATE, rate_flag)

    def set_range (self,range_flag):
        print (bus.scan())
        value = bus.readfrom_mem(self.address,self.DATA_FORMAT,1)
        value=ord(value)
        value &= ~0x0F
        value |= range_flag
        value |= 0x08
        bus.writeto_mem(self.address, self.DATA_FORMAT, value)

    def get_axes(self, gforce=False):
        print("adrees  ",self.address)
        print("scan du bus   ",bus.scan())
        self.enable_measurement(self.MEASURE)
        resultats = b'\x00\x00\x00\x00\x00\x00'
        while resultats == b'\x00\x00\x00\x00\x00\x00' :
            resultats = bus.readfrom_mem(self.address, self.AXES_DATA, 6)
            print ("résultats    ",resultats)
        x = resultats[0] | (resultats[1] << 8)
        if x & (1 << 16 - 1):
            x = x - (1 << 16)
        y = resultats[2] | (resultats[3] << 8)
        if y & (1 << 16 - 1):
            y = y - (1 << 16)
        z = resultats[4] | (resultats[5] << 8)
        if z & (1 << 16 - 1):
            z = z - (1 << 16)

        x = x * self.SCALE_MULTIPLIER
        y = y * self.SCALE_MULTIPLIER
        z = z * self.SCALE_MULTIPLIER
        if gforce is False:
            x = x * self.EARTH_GRAVITY_MS2
            y = y * self.EARTH_GRAVITY_MS2
            z = z * self.EARTH_GRAVITY_MS2
        x = round(x, 4)
        y = round(y, 4)
        z = round(z, 4)

        self.enable_measurement(self.SOMMEIL)

        return {"x": x, "y": y, "z": z}

#////////////////////////////////////////////////////////////////////////////////////////////
# classe  des objets LSM303DLHC accélromètre/magnétomètre 3 axes
class lsm303DLHC:
    # LSM303 Address definitions
    LSM303Accel_ADDR    = 25 # ou l'inverse
    LSM303Mag_ADDR      = 30

    CTRL_REG1_A = 0x20
    CTRL_REG2_A = 0x21
    CTRL_REG3_A = 0x22
    CTRL_REG4_A = 0x23
    CTRL_REG5_A = 0x24
    CTRL_REG6_A = 0x25
    REFERENCE_A = 0x26
    STATUS_REG_A = 0x27
    OUT_X_L_A = 0x28
    OUT_X_H_A = 0x29
    OUT_Y_L_A = 0x2A
    OUT_Y_H_A = 0x2B
    OUT_Z_L_A = 0x2C
    OUT_Z_H_A = 0x2D
    FIFO_CTRL_REG_A = 0x2E
    FIFO_SRC_REG_A = 0x2F
    INT1_CFG_A = 0x30
    INT1_SRC_A = 0x31
    INT1_THS_A = 0x32
    INT1_DURATION_A = 0x33
    INT2_CFG_A = 0x34
    INT2_SRC_A = 0x35
    INT2_THS_A = 0x36
    INT2_DURATION_A = 0x37
    CLICK_CFG_A = 0x38
    CLICK_SRC_A = 0x39
    CLICK_THS_A = 0x3A
    TIME_LIMIT_A = 0x3B
    TIME_LATENCY_A = 0x3C
    TIME_WINDOW_A = 0x3D
    #Magnetic field sensing register description . . . . . . . . . . . . . . . . . . . . . . . 37
    CRA_REG_M = 0x00
    CRB_REG_M = 0x01
    MR_REG_M = 0x02
    OUT_X_H_M = 0x03
    OUT_X_L_M = 0x04
    OUT_Z_H_M = 0x05
    OUT_Z_L_M = 0x06
    OUT_Y_H_M = 0x07
    OUT_Y_L_M = 0x08
    SR_REG_M = 0x09
    MAG_IRA_REG_M = 0x0A
    MAG_IRB_REG_M = 0x0B
    MAG_IRC_REG_M = 0x0C
    TEMP_OUT_H_M = 0x31
    TEMP_OUT_L_M = 0x32
####################""""
    MAG_DEVICE_ID = 0b01000000
# Magnetometer gains
    MAGGAIN_1_3 = 0x20  # +/- 1.3
    MAGGAIN_1_9 = 0x40  # +/- 1.9
    MAGGAIN_2_5 = 0x60  # +/- 2.5
    MAGGAIN_4_0 = 0x80  # +/- 4.0
    MAGGAIN_4_7 = 0xA0  # +/- 4.7
    MAGGAIN_5_6 = 0xC0  # +/- 5.6
    MAGGAIN_8_1 = 0xE0  # +/- 8.1

# Magentometer rates
    MAGRATE_0_7 = 0x00  # 0.75 Hz
    MAGRATE_1_5 = 0x01  # 1.5 Hz
    MAGRATE_3_0 = 0x02  # 3.0 Hz
    MAGRATE_7_5 = 0x03  # 7.5 Hz
    MAGRATE_15 = 0x04  # 15 Hz
    MAGRATE_30 = 0x05  # 30 Hz
    MAGRATE_75 = 0x06  # 75 Hz
    MAGRATE_220 = 0x07  # 220 Hz

# Conversion constants
    GRAVITY_STANDARD = 9.80665  # Earth's gravity in m/s^2
    GAUSS_TO_MICROTESLA = 100.0  # Gauss to micro-Tesla multiplier

    ###################""""""
    ACCELE_SCALE     = 2

    X                 = 0
    Y                 = 1
    Z                 = 2

    # Set up the sensor
    def __init__(self,):

        self.write_regAccel(0b01000111, self.CTRL_REG1_A)                # 0x57 = ODR=50hz, all accel axes on
        self.write_regAccel(0x00, self.CTRL_REG2_A)      # set full-scale
        self.write_regAccel(0x00, self.CTRL_REG3_A)              # no interrupt
        self.write_regAccel(0x00, self.CTRL_REG4_A)              # no interrupt
        self.write_regAccel(0x00, self.CTRL_REG5_A)              # 0x10 = mag 50Hz output rate
        self.write_regAccel(0x00, self.CTRL_REG6_A)
        self.write_regMag(0b10010000,self.CRA_REG_M)     # activation température et frequence mag à 35 Hz
        self.write_regMag(0b00100000,self.CRB_REG_M)     # +- 1,3 Gauss champ terrete 0,5 gauss
        self.write_regMag(0b00000000,self.MR_REG_M)      #  pour mesure continuelle 00000011 pour mise en sommeil
        self.lsm303mag_gauss_lsb_xy= 1100.0
        self.lsm303mag_gauss_lsb_z= 980.0

        print("toto")

    # get the status of the sensor
    def status(self):
        if self.read_regAccel(self.STATUS_REG_A) !=73:
            return -1
        return 1

    # Write data to a reg on the I2C device
    def write_regAccel(self,data,reg):
        #print("adresse écriture LSM303",self.LSM303Accel_ADDR)
        bus.writeto_mem(self.LSM303Accel_ADDR, reg, data)

    # Read data from the sensor
    def read_regAccel(self,reg):
        return bus.readfrom_mem(self.LSM303Accel_ADDR, reg,1)

    # Write data to a reg on the I2C device
    def write_regMag(self,data,reg):
        bus.writeto_mem(self.LSM303Mag_ADDR, reg, data)

    # Read data from the sensor
    def read_regMag(self,reg):
        return bus.readfrom_mem(self.LSM303Mag_ADDR, reg,1)

    # Check if compass is ready
    def isMagReady(self):
        if int.from_bytes(self.read_regMag(self.SR_REG_M),"big") & 0x03 != 0 :
            return 1
        return 0
    def dec2bin(self,d,nb):
    #"""Représentation d'un nombre entier en chaine binaire (nb: nombre de bits du mot)"""
        if d == 0:
            return "0".zfill(nb)
        if d<0:
            d += 1<<nb
        b=""
        while d != 0:
            d, r = divmod(d, 2)
            b = "01"[r] + b
        return '{:0>{w}}'.format(b, w=nb)   # en effet en micropyhton il n'y a pas de fonction zfill

    def temperature2(self):
        temp=((int.from_bytes(self.read_regMag(self.TEMP_OUT_H_M),"big")<<8)|(int.from_bytes(self.read_regMag(self.TEMP_OUT_L_M),"big")))
        if temp > 32767:  # 2's complement
            temp -= 65536
        temperature = float( 20 + ( temp >> 4 )/ 8 )
        return temperature
    # Get raw accelerometer values
    def getAccel(self):
        raw_accel=[0,0,0]
        #print("titi ",int.from_bytes(self.read_reg(self.OUT_X_H_A),"big"))
        raw_accel[0]=((int.from_bytes(self.read_regAccel(self.OUT_X_H_A),"big")<<8)|int.from_bytes(self.read_regAccel(self.OUT_X_L_A),"big"))
        raw_accel[1]=((int.from_bytes(self.read_regAccel(self.OUT_Y_H_A),"big")<<8)|int.from_bytes(self.read_regAccel(self.OUT_Y_L_A),"big"))
        raw_accel[2]=((int.from_bytes(self.read_regAccel(self.OUT_Z_H_A),"big")<<8)|int.from_bytes(self.read_regAccel(self.OUT_Z_L_A),"big"))
        #2's compiment
        for i in range(3):
            if raw_accel[i]>32767:
                raw_accel[i]=raw_accel[i]-65536

        return raw_accel

    # Get accelerometer values in g
    def getRealAccel(self):
        realAccel=[0.0,0.0,0.0]
        accel=self.getAccel()
        for i in range(3):
            realAccel[i] = round(accel[i] / math.pow(2, 15) * self.ACCELE_SCALE,3)
        return realAccel

    # Get compass raw values
    def getMag(self):
        raw_mag=[0,0,0]
        raw_mag[0]=((int.from_bytes(self.read_regMag(self.OUT_X_H_M),"big")<<8)|int.from_bytes(self.read_regMag(self.OUT_X_L_M),"big"))
        raw_mag[1]=((int.from_bytes(self.read_regMag(self.OUT_Y_H_M),"big")<<8)|int.from_bytes(self.read_regMag(self.OUT_Y_L_M),"big"))
        raw_mag[2]=((int.from_bytes(self.read_regMag(self.OUT_Z_H_M),"big")<<8)|int.from_bytes(self.read_regMag(self.OUT_Z_L_M),"big"))
        #2's compiment
        for i in range(3):
            if raw_mag[i]>32767:
                raw_mag[i]=raw_mag[i]-65536
        return raw_mag

    # Get heading from the compass
    def getHeading(self):
        magValue=self.getMag()
        print(magValue)
        heading = 180*math.atan2(magValue[self.Y], magValue[self.X])/math.pi#  // assume pitch, roll are 0
        if (heading <0):
            heading += 360
        return round(heading,3)

    def getTiltHeading(self):
        magValue=self.getMag()
        raw_accel=self.getAccel()
        if [0.0,0.0,0.0] != raw_accel:
            accXnorm= raw_accel[0]/math.sqrt(raw_accel[0]*raw_accel[0]+raw_accel[1]*raw_accel[1]+raw_accel[2]*raw_accel[2])
            accYnorm= raw_accel[1]/math.sqrt(raw_accel[0]*raw_accel[0]+raw_accel[1]*raw_accel[1]+raw_accel[2]*raw_accel[2])

            pitch = math.asin(accXnorm)
            roll = -math.asin(accYnorm/math.cos(pitch))
        #print(accelValue[Y],pitch,math.cos(pitch),accelValue[Y]/math.cos(pitch),math.asin(accelValue[Y]/math.cos(pitch)))

            xh = (magValue[0] * math.cos(pitch)) +( magValue[2] * math.sin(pitch))
            yh = (magValue[1] * math.sin(roll) * math.sin(pitch)) + (magValue[1] * math.cos(roll)) - (magValue[2] * math.sin(roll) * math.cos(pitch))
            zh = (-magValue[0] * (roll) * math.sin(pitch)) +( magValue[1] * math.sin(roll)) +( magValue[2] * math.cos(roll) * math.cos(pitch))
            heading = 180 * math.atan2(yh, xh)/math.pi
            #heading = 180 * math.atan2(yh*zh, xh*zh)/math.pi
            print("xh ", xh,"yh ",yh, "zh  ",zh)

            if (yh >= 0):
                return heading
            else:
                return (360 + heading)
#////////////////////////////////////////////////////////////////////////////////////////////
class AHT10:
    def __init__(self, address=0x38):
        self.address = address
        config = chr(0x08)+chr(0x00)
        bus.writeto_mem(self.address, 0xE1, config)  # calibration
    def temp_humid(self):
        #time.sleep(0.5)
        byt =  bus.readfrom(self.address,1)
        MeasureCmd = chr(0x33)+chr(0x00) #' #'0x00'+'0x00' #'0x33'+'0x00'
        bus.writeto_mem(self.address, 0xAC, MeasureCmd)
        #time.sleep(1)
        data = bus.readfrom_mem(self.address,0,6)
        temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        ctemp = ((temp*200) / 1048576) - 50
        print(u'Temperature: {0:.1f}°C'.format(ctemp))
        tmp = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4
        #print(tmp)
        ctmp = int(tmp * 100 / 1048576)
        print(u'Humidity: {0}%'.format(ctmp))
        return ctemp

#////////////////////////////////////////////////////////////////////////////////////////////////
#  procédure d'identification des périphérique sur le bux multiplexé, creation des objets correspondants
def initI2C():
    print("coucou")
    listebus=[]
    x=0
    p_canal=0  # pointeur sur canal du multiplexeur
    global tp_lsm303 # liste contenant les instances des périphérique jumeaux
    tp_lsm303=[]
    global p_lsm303 # liste contenant le canal du multiplexeur pour chaque objet
    p_lsm303=[]

    # et ainsi de suite pour chaque type de périphérique
    global tp_adxl345
    tp_adxl345=[]
    global p_adxl345
    p_adxl345=[]
    
    global tp_aht10
    tp_aht10=[]
    global p_aht10
    p_aht10=[]


    for x in range(8):
      time.sleep(0.01)
      p_canal= 2**x
      print("canal",p_canal)
      bus.writeto(0x70, p_canal) # ouverture canal du multiplexeur c'est un adressage sur le bit à 1
      listebus.append(bus.scan()) # on liste  les périphériques qui répondent sur  le canal ouvert
      print("adresse valide sur bus",x,listebus[x]) #
    x=0
    p_canal= 0
    for x in range(8):
      bus.writeto(0x70, 0)# reinitalisation du multiplexeur ?
      p_canal = 2**x
      bus.writeto(0x70, p_canal)# ouverture d'un canal du multiplexeur
      print ("attention  ",x,p_canal)
      print ("listebus ", listebus[x], listebus[x][0])
      if listebus[x][0]==25 and listebus[x][1]==30 : # on reconnait le périphérique par son (ses) adresses I2c
            objet=lsm303DLHC()
            #time.sleep(0.1)
            tp_lsm303.append(objet)  # le périphérique est rangé dans la liste de ses jumeaux
            p_lsm303.append(p_canal) # et son canal est rangé dans la liste des index
            print("canal LSM303  ",p_lsm303)
      elif listebus[x][0] == 83:
          objet = ADXL345()
          #time.sleep(0.1)
          tp_adxl345.append(objet)
          p_adxl345.append(p_canal)
          print("canal adxl345 0 ",p_adxl345)
      elif len(listebus[x])>2:   # bizarre  il a une instablité du scanavec adresse parasite  8 ??
          print(listebus[x], listebus[x][0], "CAS PARTICULIER")
          if listebus[x][1] == 83:
            objet = ADXL345()
            #time.sleep(0.1)
            tp_adxl345.append(objet)
            p_adxl345.append(p_canal)
            print("canal adxl345  1 ",p_adxl345)
      elif listebus[x][0] == 56:
          print ("JE DOIS PASSER ICI")
          objet = AHT10()
          #time.sleep(0.1)
          tp_aht10.append(objet)
          p_aht10.append(p_canal)
          print("canal aht10 0 ",p_aht10)
    # il sera possible de ne pas traiter ainsi le canal 7 (le 8ème) qui recevra tous les périphériques sans jumeaux !

    print ("et voilà")

#  interrogration d'un périphérique  LSM303 sur le bus I2C multiplexé n_capteur variable de 0 à 7
# et pointe sur l'objet choisi (il faut donc connaître son plan de branchement)
def interrogation_lsm303(n_capteur):
    bus.writeto( 0x70, (p_lsm303[n_capteur])) # ouverture du canal du multiplexeur
    #time.sleep(0.1)
    print("accélération",tp_lsm303[n_capteur].getRealAccel())
    while True:
        print("*")
        if tp_lsm303[n_capteur].isMagReady():
            break
    print(tp_lsm303[n_capteur].getHeading())
    print (tp_lsm303[n_capteur].getTiltHeading())
    print("température2 ", tp_lsm303[n_capteur].temperature2())
    return (tp_lsm303[n_capteur].getHeading())

#  interrogration d'un périphérique  ADXL345 sur le bus I2C multiplexé n_capteur variable de 0 à 7
# et pointe sur l'objet choisi (il faut donc connaître son plan de branchement)
def interrogation_adxl345(n_capteur):   #utliser comme capteur d'inclinaison du panneau solaire
    print("canal de l'adxl345  ",p_adxl345[n_capteur])
    bus.writeto( 0x70, (p_adxl345[n_capteur])) # ouverture du canal multiplexeur ad oc
    axes = tp_adxl345[n_capteur].get_axes(True)
    print ("axes    ",axes)
    print()
    return axes
    
def interrogation_aht10(n_capteur):
    #print("canal de l'aht10  ",p_aht10[n_capteur])
    bus.writeto( 0x70, (p_aht10[n_capteur])) # ouverture du canal multiplexeur ad oc
    temperature = tp_aht10[n_capteur].temp_humid()
    return temperature
    #print ("temp_humid    ",temp_humid)
    #print()
    

def incli(n_capteur):
     global mesureincli,mesureinclinterval
     mesureinclinterval=[0,0,0]
     axes = tp_adxl345[n_capteur].get_axes(True)
     axez=-(axes.get('z')) # le signe - decoule de l'implantation physique de l'accéloromètre
     if axez>1:
         axez=1  # il semble pour une raison x que l'accéloromètre envoi un valeur parfois sup à 1 ?
     print("axez",axez)
     mesur=(((math.acos(axez))/(2*(math.pi))*360)-1.5)
     mesureincli=math.floor(mesur)
     mesureinclinterval=[mesureincli-1,mesureincli,mesureincli+1]
     #print (mesureincli,"//",mesureinclinterval)
     return mesureinclinterval,mesureincli

# problème en suspend error bus I2C lorsqu'on réadresse le adxl345 !
# quand on relance le bus Ic2 sans avoir reset (couper l'alimentation du multiplexeur)
# c'est donc la négligence du reset du multiplexeur qui pose problème car il garde sans doute
# dans un tampon l'état du bus I2C lors de l'interruption du programme. Dans la mesure
#où lors du sommeil profond le wipy coupe l'alimentation (raspi,arduino, multiplexeur) la
# la réinitialisation du bus i2c semble assuré (contrairement à la phase de test
# où un machine.reset() du wipy n'interrompt pas l'alimentation de "travail")



#//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#              Programme Principal

uart.write("début de boucle")
uart.write("\r\n")
initI2C()
peto="" # variable de la requete venant de Raspi
direchose="" # variable des réponse à transmettre au raspi
while True :  
  if direchose != "" and direchose !="\r\n":
    print("DIRE CHOSE   ",direchose)
    emettre() #pour éponse du wipy aux requêtes raspi
    #ou alerte par exemple cupure alim imminente
  recevoir()  # pour suivre les requêtes du raspy et les traiter   
  voltbat=controlebatterie()
  voltcircuit=controlecircuit()
  drapeaupanne,coupealim,alimflag,direchose,instantmarche,instantarret=gestionalimentation(drapeaupanne,coupealim,alimflag,direchose,instantmarche,instantarret,charge, voltbat, voltcircuit)
  if coupealim == True:
     coupealim,alimflag=coupurealimentation(direchose,coupealim,instantmarche,alimflag)
     if coupealim==True:
         uart.write(b"flag "+"  "+str( alimflag))
         uart.write(b"\r\n")
         #sommeil()
  
  # print("çaboucle")
  #print ("interrogation du 345    //////////////////////////////////////////")
  #interrogation_adxl345(0)
  # print(incli(0))
  # print(" ")
  # print ("interrogation du 303 N°1   //////////////////////////////////////////")
  # interrogation_lsm303(0)
  # print(" ")
  # print ("interrogation du 303 N°2   //////////////////////////////////////////")
  # interrogation_lsm303(1)
  # print(" ")
  # print ("interrogation du ath10   //////////////////////////////////////////")
  # interrogation_aht10(0)

  

#////////////////////////////
