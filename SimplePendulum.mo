within ;
model SimplePendulum
  import SI= Modelica.SIunits;
  constant SI.Acceleration g = 9.81;
  parameter SI.Length L( min=0)= 1;
  SI.Angle Theta(start=0.1,fixed=true);
  SI.AngularVelocity ThetaDot;
equation
  ThetaDot= der(Theta);
  der(ThetaDot)=-(g/L)*sin(Theta);
end SimplePendulum;
