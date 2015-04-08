$fs=0.01;


//This is the normal hole in the middle of the bean bag toss game
cylinder(h=1, r1=6/2*25.4, r2=6/2*25.4);
difference(){
	cutout(expand=6);
	cutout();
	}
module cutout(height=6, rChannelInner =8.25/2*25.4, rChannelOuter=9.25/2*25.4, rRxBoard=25.4, rTxLed=12.5, expand=0){
	//height: height of extrusion
	//rChannelOuter: outer radius of the routing ring cutout (in.)
	//rChannelInner: inner radius of the routing ring cutout
	//rRxBoard: radius of the RX and uC board cutout
	//rTxLed: radius of the TX LED cutout
	//expand: the desired wall thickness
  channelWidth = (rChannelOuter - rChannelInner) / 2;
	rChannelCenter = (rChannelOuter + rChannelInner)/ 2;
	difference(){
		cylinder(h=height, r=rChannelOuter+expand);
		cylinder(h=height, r=rChannelInner-expand);
		}

	//Receiver cutouts
	for(i=[0:2]){
    translate([rChannelCenter*cos(i*120),rChannelCenter*sin(i*120),0]){
      cylinder(h=height,r=rRxBoard+expand);
			}
		}

	//Emitter cutouts
	for(i=[0:2]){
    translate([(rChannelInner-rTxLed/2)*cos(i*120+60),(rChannelInner-rTxLed/2)*sin(i*120+60),0]){
        cylinder(h=height, r=rTxLed+expand);
				}
		}

	//micro cutout
	translate([-(rChannelCenter+rRxBoard),0,0]){
    cylinder(h=height,r=rRxBoard+expand);
		}
	}
