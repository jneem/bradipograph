EyeCenter = [40, -20];
StrokeWidth = 5;
EyeInnerRadius = 11;
EyeRadius = 17 + StrokeWidth / 2;
NoseAboveEyeCenter = -5;
HeadRadius = 70;
StrokeHeight = 8;
HeadTranslation = -45;
ChinTranslation = -38;
StringHeight = 6;
StringDiameter = 0.3;
StringCutoutAngle = 10;

StepperScrewDistance = 35;
StepperScrewOffset = 8;
StepperScrewAngle = 22.5;

// Stepper driver
DriverWidth = 31.4;
DriverHeight = 35.4;
DriverPosition = [33, -80];
DriverScrewPositions = [
    DriverPosition + [2.54, 2.54],
    DriverPosition + [DriverWidth, 0] - [2.54, -2.54],
];

ScrewLocations = [
    [0, HeadRadius + HeadTranslation],
    [0, HeadRadius + HeadTranslation - 17],
    [17, 2.5],
    [-17, 2.5],
    [50.4, 2.5],
    [-50.4, 2.5],
];

NoseArcXOffset = sqrt(4 * EyeRadius^2 - (EyeRadius - NoseAboveEyeCenter)^2);
NoseArcYOffset = EyeRadius - NoseAboveEyeCenter;
NoseArcCenter = EyeCenter - [NoseArcXOffset, NoseArcYOffset];
NoseArcAngle = atan2(NoseArcYOffset, NoseArcXOffset);

BrowRadius = ((EyeCenter[1] - StrokeWidth/2)^2 + EyeCenter[0]^2 - EyeRadius^2) / (2 * (- EyeCenter[1] + StrokeWidth/2 - EyeRadius));
BrowAngle = atan2(EyeCenter[0], BrowRadius - (-EyeCenter[1] - StrokeWidth/2));

use <wheel.scad>;

$fn=60;

function cat(L1, L2) = [for (i=[0:len(L1)+len(L2)-1]) 
                        i < len(L1)? L1[i] : L2[i-len(L1)]];

function rev(L) = [for (i=[1:len(L)]) L[len(L)-i]];
 
phi = atan2(EyeCenter[1], EyeCenter[0]);
theta = function(r) asin(r / norm(EyeCenter));
s = function(r) sqrt(EyeCenter[0] * EyeCenter[0] + EyeCenter[1] * EyeCenter[1] - r * r);
stringDetachPoint = function(r) [s(r) * cos(theta(r) - phi), -s(r) * sin(theta(r) - phi)];


module strokeCircle(radius, startAngle, endAngle, strokeWidth) {
    step = 0.299;
    points = [for (t = [startAngle:step:(endAngle-0.001)]) [radius * cos(t), radius * sin(t)]];
        
    offset(r=strokeWidth/2)
    polygon(cat(points, rev(0.99999 * points)));
}

module ScrewSupport(h) {
    cylinder(h=h, r=2.5);
}
module TightScrewHole() {
    cylinder(h=100, r=1.4, center = true);
}
module LooseScrewHole() {
    cylinder(h=100, r=1.5, center = true);
}


module EyeSocket() {
        linear_extrude(StrokeHeight)
        strokeCircle(EyeRadius, 180 + NoseArcAngle, 360 + 90 - BrowAngle, StrokeWidth);
}

module StepperMount() {
    rotate(StepperScrewAngle) {
            translate([-StepperScrewDistance/2, -StepperScrewOffset]) TightScrewHole();
            translate([StepperScrewDistance/2, -StepperScrewOffset]) TightScrewHole();
    }
}

module RightEye() {
    translate(EyeCenter) {
        //Wheel();
        EyeSocket();
    }
    translate(NoseArcCenter) {
        linear_extrude(StrokeHeight)
        strokeCircle(EyeRadius, NoseArcAngle, 90, StrokeWidth);
    }
}

module Forehead2d() {
    difference() {
        intersection() {
            translate([0, HeadTranslation])
            circle(HeadRadius + StrokeWidth / 2);
        
            translate([-250, 0])
            square([500, 500]);
        }
        
        translate(EyeCenter) circle(EyeRadius - StrokeWidth / 2);
        mirror([1, 0, 0]) translate(EyeCenter) circle(EyeRadius - StrokeWidth / 2);

    }
}

module Head() {
    difference() {
        union() {
            
        linear_extrude(StrokeHeight)
        translate([0, -BrowRadius+StrokeWidth/2])
        strokeCircle(BrowRadius, 90 - BrowAngle - 0.5, 90 + BrowAngle + 0.5, StrokeWidth);
            difference() {
                union() {
                    linear_extrude(StrokeHeight)
                    Forehead2d();
            
                    // Head outline
                    translate([0, HeadTranslation]) {
                        linear_extrude(StrokeHeight)
                        intersection() {
                            strokeCircle(HeadRadius, 0, 360, StrokeWidth);
                            
                            translate([0, ChinTranslation + 100 - StrokeWidth/2])
                            square([200, 200], center = true);
                        }
                        
                        linear_extrude(StrokeHeight)

                        intersection() {
                            translate([0, ChinTranslation])
                            square([2 * HeadRadius, StrokeWidth], center = true);
                            circle(HeadRadius + StrokeWidth / 2);
                        }
                    }
                }
        
                translate([0, 0, StrokeHeight / 2]) linear_extrude(StrokeHeight) offset(0.1) Forehead2d();
            }
            for (p=ScrewLocations) {
                translate(p) ScrewSupport(StrokeWidth/2 + 1);
            }
        }
        
        for (p=ScrewLocations) {
            translate(p) TightScrewHole();
        }
    }
}

module Forehead() {
    difference() {
        union() {
            linear_extrude(StrokeWidth / 2 - 1) Forehead2d();
            for(p=ScrewLocations) {
                translate(p) ScrewSupport(StrokeWidth/2 - 1);
            }
        }
        
        for(p=ScrewLocations) {
            translate(p) TightScrewHole();
        }
        translate([0, 0.05, -1]) StringGuide();
        translate([0, -0.2, -1]) StringGuide();

        cube([2*StringDiameter, 2*StringDiameter, 10], center = true);
    }
}


module RightStringGuide() {
    eps = 0.15;    
    h = StrokeHeight - StringHeight - StringDiameter;

    module Guide() {
        hull() {
            translate([StringDiameter/2 + eps, -2*h/3, -eps]) sphere(eps);
            translate([StringDiameter/2 + eps, eps, -h/2]) cylinder(h=h, r=eps, center = true);
            translate([5, eps, -h/2]) cylinder(h=h, r=eps, center = true);

        }
    }
    
    translate([0, 0, StrokeHeight]) Guide();
    

}

module StringGuide() {
    RightStringGuide();
mirror([1, 0, 0]) RightStringGuide();
}

module RightStringCutout() {
    eps = 0.15;    
    translate([0, StrokeWidth/2 - 0.01, StrokeHeight + StringHeight])
    cube([StringDiameter + eps, StrokeWidth, 2*StrokeHeight], center = true);
    
    translate([0, eps, StringHeight])
    linear_extrude(StrokeHeight)
    polygon([[0, 0], [0, StrokeWidth * 1.1], [StrokeWidth * 1.1 / tan(StringCutoutAngle), StrokeWidth * 1.1]]);

}

module StringCutout() {
    RightStringCutout();
    mirror([1, 0, 0]) RightStringCutout();
}

module Nose() {
    module Plank() {
        translate([0, StrokeWidth / 2])
        intersection() {
            rotate([108.5, 0, 0]) translate([0, 20, 0.75]) {
                cube([15, 40, 1.5], center = true);
            }
            translate([0, 0, 50]) cube([100, 100, 100], center = true);
        }
    }
    
    translate([0, EyeRadius + NoseArcCenter[1]]) {
        Plank();
        
        difference() {
            union() {
                // Bridge connecting the nose to the eyes
                linear_extrude(StrokeHeight)
                square([NoseArcCenter[0] * 2, StrokeWidth], center=true);
                
                // Cylinder supporting the nose
                translate([0, -8, 0]) cylinder(50, r=StrokeWidth/2);
        
                // Bridge connecting the nose to the cylinder
                bridge = 8;
        
                linear_extrude(StrokeHeight)
                offset(r=StrokeWidth/2)
                translate([0, - bridge / 2])
                square([0.01, bridge], center=true);
            }
            
            hull() {
                translate([0, 0, 0.1]) Plank();
                translate([0, 15, -1]) Plank();
                translate([0, 0, 20]) Plank();


            }
        }

    }
}

module DriverScrewSupports() {
    module Right() {
        for(p=DriverScrewPositions) {
            translate(p) ScrewSupport(StrokeHeight);
        }
        
        linear_extrude(StrokeHeight)
        offset(r=StrokeWidth/2)
        translate(DriverScrewPositions[0] + [0, -2.5])
        square([0.01, 5], center = true);
    }
    
    Right();
    mirror([1, 0, 0]) Right();
}

module DriverScrewHoles() {
    module Right() {
        for(p=DriverScrewPositions) {
            translate(p) TightScrewHole();
        }
    }
    
    Right();
    mirror([1, 0, 0]) Right();
}


module Positive() {
RightEye();
mirror([1, 0, 0]) RightEye();



Head();
Nose();
    StringGuide();
    
    DriverScrewSupports();
    //StringCutout();
};

module Negative() {
    translate(EyeCenter) StepperMount();
    mirror([1, 0, 0]) translate(EyeCenter) StepperMount();
    DriverScrewHoles();
    StringCutout();
    
    // Extra screw hole at the bottom, maybe for a battery?
    translate([0, ChinTranslation + HeadTranslation]) TightScrewHole();
    
    for (p=ScrewLocations) {
        translate(p) TightScrewHole();
    }
}

difference() {
    Positive();
    Negative();
}

translate([120, 0, StrokeWidth/2 + 1]) Forehead();

translate(DriverPosition) cube([31.4, 35.4, 1]);


