import pygmaps



topLeftX =  42.746616
topLeftY = -73.715328


mymap = pygmaps.maps(42.729358, -73.674453, 32)


mymap.addpoint(42.80000, -73.680000,"#0000FF")

#mymap.addpath(path,"#00FF00")

mymap.draw('./test2.HTML')
