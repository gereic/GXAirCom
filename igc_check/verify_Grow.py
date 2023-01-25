#!/usr/bin/python
import sys, getopt
import hashlib

def main(argv):
    inputfile = ''
    pkfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:p:",["ifile=","pkfile="])
    except getopt.GetoptError:
        print ('verify_Grow.py -i <file.igc> -p <tzinstruments.pk>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print ('verify_Grow.py -i <file.igc> -p <tzinstruments.pk>')
            sys.exit()
        elif opt in ("-i", "--igcfile"):
            inputfile = arg
        elif opt in ("-p", "--pkfile"):
            pkfile = arg
    try:
        print ('Input file is ', inputfile)
        f = open(inputfile,'r')
        data = [ r.rstrip() for r in f.readlines() if r[0]=='B' or r[0] == 'G' or r[0:2]=="HF"]
        hfrows = [ d for d in data if d[0:2]=="HF"]
        gpsdata = [ d for d in data if d[0] =='B' ]
        grow = data[-1]
        f.close()

        f =  open(pkfile,'r')
        pk = f.read().rstrip()
        pk = pk.split(' ')[-1].replace('"','')
        f.close()
        encoded=pk.encode()

        result = hashlib.sha256(encoded)

        for h in hfrows:
            print (h)
            s = result.hexdigest() + h
            result = hashlib.sha256(s.encode())

        for b in gpsdata:
            s = result.hexdigest() + b
            result = hashlib.sha256(s.encode())

        print(F"Original grow: G-{grow[1:]}")
        print(F"Computed grow: G-{result.hexdigest()}")

        if ("G"+result.hexdigest() == grow):
            print("SHA256 Match, Verified Correctly!")
        else:
            print("SHA256 NOT VERIFIED!")
    except FileNotFoundError:
        print('igc or pk files missing arguments or not found.')
        print ('verify_Grow.py -i <file.igc> -p <tzinstruments.pk>')

if __name__ == "__main__":
   main(sys.argv[1:])