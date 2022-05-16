// Belt pulley is http://www.thingiverse.com/thing:3104 by GilesBathgate
// GPLV3
// Made parametric by droftarts September 2011

/**
 * @name Pulley
 * @category Printed
 * @using 1 x m3 nut, normal or nyloc
 * @using 1 x m3x10 set screw or 1 x m3x8 grub screw
 */

module pulley()
{

m3_dia = 3.4;		// 3mm hole diameter
m3_nut_hex = 1;		// 1 for hex, 0 for square nut
m3_nut_flats = 5.9;	// normal M3 hex nut exact width = 5.5
m3_nut_depth = 4.4;	// normal M3 hex nut exact depth = 2.4, nyloc = 4
motor_shaft = 5.2;	// exact NEMA17 motor shaft size = 5

retainer = 1;		// Belt retainer end, 1 for Yes, 0 for No
idler = 0;			// 0 = No, 1 = Yes; fills in dish of retainer, if required

teeth = 8;			// Number of teeth, standard Mendel = 8
pulley_t_ht = 12;	// length of toothed part of pulley, standard = 12
pulley_b_ht = 8;		// pulley base height, standard = 8
pulley_b_dia = 20;	// pulley base diameter, standard = 20
no_of_nuts = 1;		// number of captive nuts required, standard = 1
nut_angle = 90;		// angle between nuts, standard = 90

// calculated constants

m3_nut_points = 2*((m3_nut_flats/2)/cos(30)); // This is needed for the nut trap
teeth_dia = ((1.591 * pow(teeth,1.064)) / (0.6523 + pow(teeth,1.064)))*teeth; // Outside diameter of teeth
echo (teeth_dia);
echo (teeth_dia/2-1.3); // radius of shaft under teeth

echo((teeth_dia+teeth_dia/2-1.3)/2);

polygon_points = [[0,0],[1+teeth_dia/2-2.5,0],[4+teeth_dia/2-2.5,3],[4+teeth_dia/2-2.5,4],[2,4],[0,4]];
polygon_paths = [[0,1,2,3,4,5,0]];
	module spur()
	{
		linear_extrude(height=pulley_t_ht) polygon([[-1.7,-1],[-1.7,1],[0,0.7],[0,-0.7]],[[0,1,2,3,0]]);
	}

	module retainer()
	{
		rotate_extrude($fn=teeth*4) polygon(polygon_points,polygon_paths);
	}

difference()
 {	 
 	union()
 	{
 		//base

		if ( pulley_b_ht < 2 ) { echo ("CAN'T DRAW PULLEY BASE, HEIGHT LESS THAN 2!!!"); } else {
 			rotate_extrude($fn=pulley_b_dia*2)
 			{
 					square([pulley_b_dia/2-1,pulley_b_ht]);
 					square([pulley_b_dia/2,pulley_b_ht-1]);
 					translate([pulley_b_dia/2-1,pulley_b_ht-1]) circle(1);
 			}
		}

    	//shaft
    	//cylinder(r=teeth_dia/2-1.3,h=pulley_b_ht + pulley_t_ht, $fn=teeth);
    	translate(v=[0,0,pulley_b_ht]) cylinder(r=teeth_dia/2-1.3,h=pulley_t_ht, $fn=teeth);
    	//spurs
	for(i=[1:teeth]) rotate([0,0,i*(360/teeth)])
	translate([teeth_dia/2,0,pulley_b_ht]) spur();

	//belt retainer end
	if ( retainer > 0 ) {
        translate ([0,0,pulley_b_ht + pulley_t_ht - 2]) retainer();
        mirror(v=[0,0,1]) translate ([0,0, -(pulley_t_ht - 2)]) retainer();
	if ( idler > 0 ) {translate ([0,0,pulley_b_ht + pulley_t_ht]) cylinder(r=teeth_dia/2,h=2,$fn=teeth*2);}}

	}
   
	//shaft hole
    translate([0,0,-1])cylinder(r=motor_shaft/2,h=pulley_b_ht + pulley_t_ht + 4,$fn=motor_shaft*4);
    		
    //captive nut and grub holes

	if ( pulley_b_ht < m3_nut_flats ) { echo ("CAN'T DRAW CAPTIVE NUTS, HEIGHT LESS THAN NUT DIAMETER!!!"); } else {
	if ( (pulley_b_dia - motor_shaft)/2 < m3_nut_depth + 3 ) { echo ("CAN'T DRAW CAPTIVE NUTS, DIAMETER TOO SMALL FOR NUT DEPTH!!!"); } else {

		for(j=[1:no_of_nuts]) rotate([0,0,j*nut_angle])
		translate([0,0,pulley_b_ht/2])rotate([90,0,0])

		union()
		{
			//entrance
			translate([0,-pulley_b_ht/4-0.5,motor_shaft/2+m3_nut_depth/2+1]) cube([m3_nut_flats,pulley_b_ht/2+1,m3_nut_depth],center=true);

			//nut
			if ( m3_nut_hex > 0 )
				{
					translate([0,0.25,motor_shaft/2+m3_nut_depth/2+1]) rotate([0,0,30]) cylinder(r=m3_nut_points/2,h=m3_nut_depth,center=true,$fn=6);
				} else {
					translate([0,0.25,motor_shaft/2+m3_nut_depth/2+1]) cube([m3_nut_flats,m3_nut_flats,m3_nut_depth],center=true);
				}

			//grub hole
			rotate([0,0,22.5])cylinder(r=m3_dia/2,h=pulley_b_dia/2+1,$fn=8);
		}
	}}
 }
   
}
pulley();