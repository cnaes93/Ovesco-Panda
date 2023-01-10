import numpy as np
import time



matrixAusText = np.loadtxt("example1.txt", dtype=int) #

#print(matrixAusText)#hier wird die Matrix gedruck
anzahlZeilen = len(matrixAusText) # das gibt mir di anzahl der zeilen
#print(anzahlZeilen, "das ist die anzahl der zeilen") # das druck die anzahl der zeilen

zuAddieren = np.eye(anzahlZeilen) #das erzeugt eine matrix mit 1er auf der querachse. Zeilen und spaltenanazl ist = Länge der eingelesenen matriy
zuAddieren1 = zuAddieren[zuAddieren == 1] = 1000
#print(zuAddieren, "das ist die neue 1er matrix")


neueMatrix = np.append(matrixAusText, zuAddieren, axis=1)
#print(neueMatrix, "das ist die neue matrix mit der er im Anhang")

anzahlSpalten = 3
Absatand = 0
j = 1 # j soll die spalte sein
i = 0

Zeile = 0 # da lege ich die zeile fest die bearbeitet wird


while Zeile < anzahlZeilen: #

    while i < anzahlZeilen: # das soll die tabelle mit den abständen aufstellen
        #print(i) # ich zähle i (zeile) von 0 hoch
        deltaX0i = matrixAusText[Zeile, 1] - matrixAusText[i, 1] # rechne zeile 0 spalte j - zeile i spalte j
        deltaY0i = matrixAusText[Zeile, 2] - matrixAusText[i, 2]
        Absatand = deltaX0i^2 + deltaY0i^2
        Absatand = Absatand**0.5
        Absatand = round(Absatand, 2)

        neueMatrix[Zeile,i+3] = Absatand # HIer wird i als Spalte der neuen matrix und Zeile als zeile verwendet
        i = i+1 # ab hier wird die nächste zeile subtrahiert.
        #print(neueMatrix)
        Zeile = Zeile + 1 # hier zählt die zeile hoch. Jeztt wird die nächste zeile bearbeitet
        i = 0 # i ist die zeile die zum subtrahieren beutzt wird



print(neueMatrix)