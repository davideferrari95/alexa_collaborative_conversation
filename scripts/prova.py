toolLocation={
    'part_base':["red",False],
    'part2':["red",False],
    'part2':["red",False],
    'part4':["red",False],
    'part5':["red",False],
    'screw_m4':["red",False],
    'allenkey 8':["green",False],
    'allenkey 6':["yellow",False],
    'bottle':["yellow",False],
    'cross screwdrivers':["green",False],
    #'flat screwdriver':[[],False,True],
    'multifunction screwdrivers':["yellow",False],
    'wrench':["yellow",False],
    'allen screws':["green",False],
    'cross screws':["green",False]
}



tool ="flat screwdrivers"
print(tool)
if toolLocation[tool][1]==True:
    location="the " + tool + " ,is already taken"
else:    
    if tool not in toolLocation.keys() and "screwdriver" in tool:
        pit = tool.split()[0]
        location = "there isn't the "+ tool +", but there is the "+ pit +" pit for the multyfunction screwdriver"
    elif tool not in toolLocation.keys() and "screwdriver" not in tool:
        location = "there isn't the "+ tool 
    else:
        location = "the " + tool + " is in the "+ toolLocation[tool][0] + " area"
        global lastToolSelected
        lastToolSelected=tool
        print (lastToolSelected)
print(location)