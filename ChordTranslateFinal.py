''' This code translates an entire song's guitar chords into a set of instructions for the DC motors.
This code first opens the desired song text file which contains the chords used in order line by line ex:
                    "
                    Am
                    C
                    G
                    Am
                    "
Here we compile a list of the chords that are used. Then we go and find these chords and then cross-referemce them with "chords.txt" file that breaks down all necessary
chords in chordbox notation, ex: Am:X02210 where X means don't play, 0's mean strum but there's no "fingers" pressing on them and the numbers indicate the
fret where the finger pushes that string down. Now, once we have this info we will map each "finger" (motor) position and write that to a new created text
file in the form: A150 meaning motor A move to position 150mm etc...

The output of this python script is a text file that will then be downloaded onto a microSD card that the c++ code on the Arduino Mega 2560 will open and 
use to position the motors.
 '''

#Defining the files
import sys, os
songFile = open("C:(****PATH*****)\\riptide_chords.txt","r")
chordsFile = open("C:(****PATH*****)\\chords.txt","r")
savePath = ("C:(****PATH*****)\\Translated_songs")

#VARIABLES
songTitle = os.path.basename(songFile.name)
songTitle = songTitle[:len(songTitle)-4]
usedChords = []
chordboxes = []
motors = ['A','B','C','D','E','F']
positions = []
#These are the measured distances of the frets on my guitar
fretDist = {'1':1,'2':34,'3':66,'4':97,'5':127,'6':155,'7':182,'8':208,'9':234,'10':254,'11':275,'12':297,'13':316,'14':331,'15':348,'16':364} #dist in mm from top of neck
prevPos = [0]*6


#This fucntion writes the translated positions to the new file 
def positioning(chord,first):
    for i in range(0,6):
        if first and (chord[i] == 'X' or chord[i] == '0'):
            motorPos = '1'
        elif (chord[i] == 'X' or  chord[i] == '0') and not first:
            motorPos = "U" + str(prevPos[i])  # for unpressed
        else:
            motorPos = fretDist[chord[i]]
            motorPos = "P" + str(motorPos)  #to indicate pressed
        motorPos = motors[i] + motorPos
        translatedSong.write(motorPos+"\n")
        prevPos[i] = motorPos[1:len(motorPos)-1]
    #translatedSong.write("\n")
    first = True
        


def main():
    #compiling list of all chords used
    songChords = songFile.readlines() #returns a list composed of each eleemnt per line. NOTE that each note ends w/ line break "\n"
    for i in range(0,len(songChords)):
        songChords[i] = songChords[i][0:len(songChords[i])-1] #getting rid of "\n"
        if songChords[i] not in usedChords:
            usedChords.append(songChords[i])
    print(usedChords)

    #finding each chord's corresponding chorbox notation
    chordsText = chordsFile.readlines()
    for n in range(0,len(usedChords)):
        for k in range(0,len(chordsText)):
            if usedChords[n] in chordsText[k]:
                chordboxes.append(chordsText[k][len(usedChords[n])+1:len(chordsText[k])-1])
    print(chordboxes)

    #calling the function that positions the motors 
    first = True
    for r in range(0,len(songChords)):
        indi = usedChords.index(songChords[r])
        positioning(chordboxes[indi],first)
        first = False


if __name__=="__main__":

    fullname = os.path.join(savePath,f"Translated_{songTitle}.txt") #saves it where we want it to
    translatedSong = open(fullname,"w+")
    main()
    songFile.close()
    chordsFile.close()
#    translatedSong.write("END \n")
    translatedSong.close()