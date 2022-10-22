$fn = 90;

$y = 22.5;
$x = 25;
$z = 7;
$t = 1.2;
$b = 3;
$bz = 0.8;

$cx = 10;
$cy = 15;

$k_d = 3.8;

module inner($_z = $z * 2) {
    cube([$x - $t ,$y -$t, $_z], center = true);
    translate([0, -$y/2 - $cy/2 + $t, 0]) cube([$cx-$t, $cy+$t, $_z], center = true);
}

module bottom($_z = $bz) {
    cube([$x + $b*2,$y + $b*2, $_z], center = true);
    translate([0, -$y/2 - $cy/2, 0]) cube([$cx + $b*2, $cy +$b*2, $_z], center = true);
}

module outer() {
    cube([$x,$y, $z], center = true);
    translate([0, -$y/2 - $cy/2, 0]) cube([$cx, $cy, $z], center = true);
}


module p1() {    
    difference() {
        union() {
            translate([0,0,-$z/2 + $bz/2]) bottom();
            outer();
        }
        inner();
        translate([0,0,-($z-$k_d)/2]) rotate([90,0,0]) cylinder(h = $y + $cy, d = $k_d);
    }
}

module p2() {
    inner(1);
}

p1();
//p2();
