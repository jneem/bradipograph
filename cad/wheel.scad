// Diameter of (the round part of) the shaft.
ShaftDiameter = 5.0;

// Width of the flat part of the shaft;
ShaftFlatWidth = 3.0;

// The radius of the wheel, measured at the deepest part of the V
WheelRadius = 13;

// The outer radius of the hub
HubRadius = 5;

// Total thickness of the wheel
Thickness = 6;

// Radius of the rim cut-out for the string
StringRadius = 0.8;

$fs = 0.4;
$fa = 1;

module RimCrossSection() {
    t = Thickness / 2;
    rotate(-90)
    polygon([[0, 0], [t, t], [t, 0], [t/2, -t/2],
             [-t/2, -t/2], [-t, 0], [-t, t]]);
}

module Rim() {
    difference() {
        rotate_extrude()
        translate([WheelRadius, 0, 0]) 
        RimCrossSection();
        
        t = WheelRadius + StringRadius * sqrt(2);
        translate([t, 0, 0]) rotate([0, -45, 0]) cylinder(10, StringRadius, StringRadius, center = true);
        translate([t, 0, 0]) rotate([0, 45, 0]) cylinder(10, StringRadius, StringRadius, center = true);
    }
}

module Shaft() {
    intersection() {
        cylinder(Thickness * 2, ShaftDiameter / 2, ShaftDiameter / 2, center = true);
        cube([ShaftFlatWidth, ShaftDiameter * 2, Thickness * 2], center = true);
    }
}

module Hub() {
    difference() {
        cylinder(Thickness, HubRadius, HubRadius, center = true);
        Shaft();

    }
}

module Spoke2d() {
    // Distance from the end of the spoke to the origin. Should be inside the hub
    // somewhere.
    dist = ShaftDiameter / 2 + (HubRadius - ShaftDiameter / 2) / 2;
    h = WheelRadius + Thickness / 2 - dist;
    difference() {
        translate([h/2 + dist, 0])
        square([h, Thickness], center = true);
        
        translate([WheelRadius + 0.2, 0])
        minkowski() {
            translate([Thickness / 2, 0]) square([Thickness, 0.1], center = true);
            RimCrossSection();
        };
    }
}
module Spoke() {
    rotate([90, 0, 0])
    linear_extrude(1, center = true) Spoke2d();
}

module Wheel() {
for (deg = [0:60:360]) {
    rotate([0, 0, deg+30]) Spoke();
}

Hub();
Rim();
}

Wheel();