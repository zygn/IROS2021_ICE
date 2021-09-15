import csv

sx =  0.8007017
sy = -0.2753365
stheta: 4.1421595

f = open ('wp_SOCHI.csv','r')
g = open ('transfer_wp_SOCHI.csv','w')

rdr = csv.reader(f)
wr = csv.writer(g)

lines = []

for line in rdr:
    line[0] = float(line[0]) + sx
    line[1] = float(line[1]) + sy 
    
    wr.writerow(line)


f.close()
g.close()