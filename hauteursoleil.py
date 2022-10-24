#!utf8
# modules à importer *
import math  #, random
import datetime
import ephem # librairie Astro


# pour installer pyEphem sous Gnu/Linux
# sudo apt-get install python-dev
# sudo apt-get install python-pip
# pip3 install ephem

#dÃ©claration des variables globales pour calcul astro
longitude="" # variable longitude
latitude="" # variable latitude

# fonction principale
def tablesuivi():
        tabh=[]  #table  heure du pointage
        tabm=[]  #table  minute du pointage
        tabe=[]  #table élévation du pointage
        taba=[]  #table azimut du pointage
        #-- calcul de la position rÃ©elle du soleil
        # paramÃ¨tres observateur
        global obs, latitude, longitude # objet observateur

        longitude="-2:76:00" # -W / +E
        latitude="47:53:00" # +N / -S
        obs=ephem.Observer() # objet global dÃ©sigant l'observateur local

        print("Longitude =" + longitude)
        print("Latitude =" + latitude)

        dt=datetime.datetime.now() # date/heure locale
        print (dt.strftime("%d %B %Y - %I:%M:%S"))

        dt=datetime.datetime.utcnow() # date/heure utc
        dt.strftime("%d %B %Y - %I:%M:%S")

        #obs.date=datetime.datetime.utcnow() # date/heure utc
        #obs.date=ephem.Date('2013/11/27 12:00') # date (A/M/D) /heure - pour test
        obs.date=datetime.date.today() # date courante
        dernierehauteur=6
        dernierazimut=0
        for i in range(24*60): # defile les minutes
                #obs.date=ephem.Date(obs.date + (ephem.hour)) # ajoute le nombre d'heures Ã�  la date
                obs.date=ephem.Date(obs.date + (ephem.minute)) # ajoute le nombre de minute Ã�  la date
                 
                obs.lat=str(latitude)
                obs.long=str(longitude)
                
                sun=ephem.Sun(obs) # objet représentant le soleil basé sur paramètres observateur
                if (((abs(int(math.degrees(sun.alt)-dernierehauteur)))>=2) or ((abs(int(math.degrees(sun.az)-dernierazimut)))>=3))and((abs(int(math.degrees(sun.alt))>=8))) :
                        #print ((obs.date),"     ",str(int(math.degrees(sun.alt))),"    ",str(int(math.degrees(sun.az))))
                        date=str(obs.date)
                        print(date)
                        heure=int(date[10:13])
                        print(heure)
                        tabh.append(heure)
                        minute=int(date[14:16])
                        print(minute)
                        tabm.append(minute)
                        tabe.append(int(math.degrees(sun.alt)))
                        taba.append(int(math.degrees(sun.az)))
                        dernierehauteur=(int(math.degrees(sun.alt)))
                        dernierazimut=(int(math.degrees(sun.az)))
                        
                else :
                        pass 
        return tabh,tabm,tabe,taba               

#--- obligatoire pour rendre code exÃ©cutable ---
if __name__ == "__main__": # cette condition est vraie si le fichier est le programme exÃ©cutÃ©
        tabh,tabm,tabe,taba =tablesuivi()# appelle la fonction principale
        print (tabh)
        print (tabm)
        print(tabe)
        print(taba)
