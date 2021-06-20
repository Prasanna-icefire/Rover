import os



def generate(lat1,long1,lat2,long2):
    msg = "curl 'https://dev.virtualearth.net/REST/V1/Routes/Driving?wp.0="+str(lat1)+","+str(long1)+"&wp.1="+str(lat2)+","+str(long2)+"&key=Ai42a3aWYoR8ArwdJurG_LPZ6LQ0MGfw5-mfUyiSQ2nd5pzoMMR4Jjni9ZkZFfoE' -o output.json"
    print(msg)
    os.system(msg)

generate(13.015816643359752, 77.66981557166832,13.016715616055903, 77.6692254856633)