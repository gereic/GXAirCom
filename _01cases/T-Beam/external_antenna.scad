$fn=36;

use <MCAD/boxes.scad>

module original() {
    import("TTGO T-Beam V1.x Oled Front 1.4.stl");
}

module proj() {
    translate([0,0,-10]) rotate([0,90,0]) original();
}


module closed_box() {
    union() {
        rotate([0,-90,0]) translate([0,0,10]) linear_extrude(height=35) difference() {
            projection(cut = true) {
                proj();
            }

            translate([-50,-52,0])square([100,50]);
        }

        difference() {
            original();
            translate([-45,0,0]) cube([35,25,30]);
        }
    }
}
//# original();

module newbox(x,y,z,r,wall=2,radius=5) {
    
    difference() {
    
        
        translate([x/2,y/2,z/2]) {
            union() {
                translate([0,-y/4,0]) cube([x,y/2+radius, z], center=true);
                roundedBox([x,y+radius,z], r, true);
            }
        }
        translate([0,-radius,wall]) cube([x,radius,z-wall]);
    }
}

box_x=35;
box_y=18;
box_z=17;
box_r=5;
translate_x=-58;
translate_y=3;
translate_z=0;
hole_d = 6.5;

difference() {
    union() {
        translate([translate_x,translate_y,translate_z]) newbox(box_x,box_y,box_z,box_r);
        closed_box();
    }
    
    translate([translate_x+2,translate_y-3,translate_z+2]) newbox(box_x-5,box_y,box_z-4,2);
    translate([translate_x + box_x - 5, box_y/2 + hole_d/2, translate_z + box_z/2]) rotate([0,90,0]) linear_extrude(height=10) circle(d=hole_d);
}


//newbox(box_x,box_y,box_z,box_r);
