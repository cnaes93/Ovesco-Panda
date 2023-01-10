import numpy as np
import time
#import pandas as pd



matrixAusText = np.loadtxt("KalibrierteFoto_angepassenteGröße.txt", dtype=float) #
MatrixOhneNase = np.delete(matrixAusText, np.where(matrixAusText==2.0)[0], axis=0) # löschen die Eingabe von Nase (Labe =2), (Zeile von den Nasen werden aus der Matrix entfernt)
#print('MatrixOhneNase = ')
#print(MatrixOhneNase)
MatrixOhneConfidensce=MatrixOnheLabel=np.delete(MatrixOhneNase,5,1) #löschen die letzte Spalte (Confidence), weil es nicht nötig ist.
MatrixOhneHöhe=MatrixOnheLabel=np.delete(MatrixOhneConfidensce,4,1) #löschen die letzte Spalte (Höhe der Box), weil es nicht nötig ist.
MatrixOhneBreite=MatrixOnheLabel=np.delete(MatrixOhneHöhe,3,1) #löschen die letzte Spalte (Bereite der Box), weil es nicht nötig ist.
#print(MatrixOhneConfidensce)
MatrixOnheLabel=np.delete(MatrixOhneBreite,0,1) # löschen die erste Spalte (Labels), weil es nicht nötig ist.
#print('MatrixOnheLabel = ')
print(MatrixOnheLabel)
#print(matrixAusText)#hier wird die Matrix gedruck
#anzahlZeilen = len(MatrixOnheLabel) # das gibt mir di anzahl der zeilen
anzahlZeilen = len(MatrixOnheLabel) # das gibt mir di anzahl der zeilen
#print(anzahlZeilen, "das ist die anzahl der zeilen") # das druck die anzahl der zeilen

zuAddieren = np.eye(anzahlZeilen) #das erzeugt eine matrix mit 1er auf der querachse. Zeilen und spaltenanazl ist = Länge der eingelesenen matriy
zuAddieren1 = zuAddieren[zuAddieren == 1] = 1000
#print(zuAddieren, "das ist die neue 1er matrix")


neueMatrix = np.append(MatrixOnheLabel, zuAddieren, axis=1)
#print(neueMatrix, "das ist die neue matrix mit der er im Anhang")
#print(neueMatrix)

anzahlSpalten = 2 
Absatand = 0
j = 1 # j soll die spalte sein
i = 0

Zeile = 0 # da lege ich die zeile fest die bearbeitet wird


while Zeile < anzahlZeilen: #

    while i < anzahlZeilen: # das soll die tabelle mit den abständen aufstellen
        #print(i) # ich zähle i (zeile) von 0 hoch
        deltaX0i = MatrixOnheLabel[Zeile, 0] - MatrixOnheLabel[i, 0] # rechne zeile 0 spalte j - zeile i spalte j
        deltaY0i = MatrixOnheLabel[Zeile, 1] - MatrixOnheLabel[i, 1]
        #print(MatrixOnheLabel[Zeile, 1],MatrixOnheLabel[i, 1])
        Abstand = deltaX0i**2 + deltaY0i**2
        Abstand = Abstand**0.5
        #Absatand = round(Absatand, 2)
        neueMatrix[Zeile,i+2] += Abstand # HIer wird i als Spalte der neuen matrix und Zeile als zeile verwendet
        i = i+1 # ab hier wird die nächste zeile subtrahiert.
        #print(neueMatrix)
    Zeile = Zeile + 1 # hier zählt die zeile hoch. Jeztt wird die nächste zeile bearbeitet
    i = 0 # i ist die zeile die zum subtrahieren beutzt wird

#print(neueMatrix)
FinalMatrix = np.delete(neueMatrix, np.where(neueMatrix < 0.09)[0], axis=0) # Löschen die Objekten mit einem Abstand kleiner als 0,09
#print(FinalMatrix)
FinalMatrix=np.delete(FinalMatrix,6,1)  # Löschen aller Abständen
FinalMatrix=np.delete(FinalMatrix,5,1)	# Löschen aller Abständen
FinalMatrix=np.delete(FinalMatrix,4,1)	# Löschen aller Abständen
FinalMatrix=np.delete(FinalMatrix,3,1)	# Löschen aller Abständen
FinalMatrix=np.delete(FinalMatrix,2,1)	# Löschen aller Abständen
print('Output')
print(FinalMatrix)

ix = 0	#Zeilennummer in matrixAusTextix (Originale Matrix aus Yolov5)
jx = 0	#Zeilennummer in FinalMatrix
AusgangMatrix = []
Z = 0

while ix < len(matrixAusText):
	while jx < len(FinalMatrix):
		if matrixAusText[ix, 1] == FinalMatrix[jx, 0] and  matrixAusText[ix, 2] == FinalMatrix[jx, 1]  :
			if Z==0:
				AusgangMatrix = [np.array(matrixAusText[ix, :])]
			else:
				AusgangMatrix =np.append(AusgangMatrix, [matrixAusText[ix, :]],axis=0)				
			Z +=1
		jx +=1
	if matrixAusText[ix, 0] ==2:
		if Z==0:
			AusgangMatrix = [np.array(matrixAusText[ix, :])]
		else:
			AusgangMatrix =np.append(AusgangMatrix, [matrixAusText[ix, :]],axis=0)
		Z +=1
	jx=0
	ix +=1

AusgangMatrix = np.delete(AusgangMatrix, np.where(AusgangMatrix==1.0)[0], axis=0)
AusgangMatrix = np.delete(AusgangMatrix, np.where(AusgangMatrix==0.0)[0], axis=0)
print('Die Ausgangsmatrix ist: ')
print(AusgangMatrix)
np.savetxt('AusgangMatrix.txt',AusgangMatrix)

