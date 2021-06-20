import json
def getCoordinates():
    with open('output.json') as data_file:   
        data = json.load(data_file)
        resourseSet = data["resourceSets"]
        dictresourceSet = resourseSet[0]
        resources = dictresourceSet["resources"]
        resourceSet = resources[0]
        resource_sub = resourseSet[0]
        routelegSet = resource_sub["resources"]
        routelegdict = routelegSet[0]
        listx = routelegdict["routeLegs"]
        dictx = listx[0]
        listy = dictx["itineraryItems"]
        #final thing for diff coordinates
        nearcoordinate = []
        listofcoordinates = []
        for i in listy:
            dicty = i
            nearcoordinate = dicty["maneuverPoint"]
            coordinates = nearcoordinate["coordinates"]
            listofcoordinates.append(coordinates)
            
    return(listofcoordinates)





    
    

