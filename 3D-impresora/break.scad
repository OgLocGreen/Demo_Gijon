
screw_d = 3;
width = 6;
depth = 12;
hight_beginning = 2;
hight_ennding = 3;

difference(){
    block();
    #translate([3, 3,-5]) cylinder(10, d=screw_d,[0,0,0], $fn = 20);
}

 module prism(l, w, h){
       polyhedron(
               points=[[0,0,0], [l,0,0], [l,w,0], [0,w,0], [0,w,h], [l,w,h]],
                center=true,
               faces=[[0,1,2,3],[5,4,3,2],[0,4,5,1],[0,3,4],[5,2,1]]

                 );
     }
 
module block(){
        cube([width,depth,hight_beginning],[0,0,0]);
        translate([0,0,hight_beginning]) prism(width, depth, (hight_ennding-hight_beginning));
    
}
