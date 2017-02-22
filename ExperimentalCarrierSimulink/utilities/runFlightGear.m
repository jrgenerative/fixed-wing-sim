% 1) You need to check that your FlightGear installation path is configured correctly below.
% 2) You need to install the 'ExperimentalCarrier' aircraft in %FG_ROOT%/data/Aircraft, e.g. C:\Program Files\FlightGear 3.4.0\data\Aircraft\ExperimentalCarrier

setenv('FG_ROOT', 'C:\Program Files\FlightGear 3.4.0\data');
currPath = pwd;
cd 'C:\Program Files\FlightGear 3.4.0'
system('bin\fgfs --aircraft=ExperimentalCarrier --fdm=network,localhost,5501,5502,5503 --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --enable-freeze --airport=KSFO --runway=10L --altitude=328 --heading=0 --offset-distance=0 --offset-azimuth=0 &')
cd(currPath);
