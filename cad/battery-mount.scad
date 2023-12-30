BatterySize = [26.2, 16.9];
WallThickness = 1;
BackOffset = 2;
BoxHeight = 15;
ScrewOffset = 2.5;
ChinDepth = 2.5;

$fn=100;

module Box() {
    translate([-BatterySize[0] - WallThickness * 1.5, 0, 0])
    cube([BatterySize[0] * 2 + WallThickness * 3, BatterySize[1] + WallThickness * 2 + BackOffset, BoxHeight]);
    
    hull() {
        translate([-BatterySize[0] - WallThickness * 1.5, -ChinDepth, BoxHeight - 0.01])
        cube([BatterySize[0] * 2 + WallThickness * 3, ChinDepth, 0.01]);
        
        translate([-BatterySize[0] - WallThickness * 1.5, 0, BoxHeight - 0.01 - ChinDepth])
        cube([BatterySize[0] * 2 + WallThickness * 3, ChinDepth, 0.01]);
    }
}

module Battery() {
    module Right() {
        translate([WallThickness / 2, WallThickness + BackOffset, WallThickness])
        cube([BatterySize[0], BatterySize[1], 50]);
    }
    
    Right();
    mirror([1,0,0]) Right();
}

module Hanger() {
    module Support() {
        translate([0, BackOffset, BoxHeight + ScrewOffset])
        rotate([90, 0, 0])
        cylinder(BackOffset, r=3);
    }
    
    hull() {
        Support();
        translate([0, 0, -5]) Support();
    }
}

module ScrewHole() {
    translate([0, 0, BoxHeight + ScrewOffset])
    rotate([90, 0, 0])
    cylinder(30, r=1.5, center = true);
}


module Positive() {

Box();
Hanger();
}

module Negative() {
    Battery();
    ScrewHole();
}

difference() {
    Positive();
    Negative();
}

//difference() {
  //  translate([-26 - 4.5, 0]) cube([26 + 26 + 9, 17 + 6, 15]);
  //  translate([1.5, 3, 3]) cube([26, 17, 15]);
  //  translate([-26 - 1.5, 3, 3]) cube([26, 17, 15]);

//}

//translate([0, 0, 5])
//difference() {
    //translate([0, -2.5, 0]) cube([14, 11, 10], center = true);
    
    //translate([0, -4.01, 2.5]) cube([8.1, 8.02, 20], center = true);


    // Screw holes for attaching to the bradipous
    //translate([0, -4, 2.5]) rotate([0, 90, 0]) cylinder (100, 0.9, 0.9, center = true, $fn=20);
//}

