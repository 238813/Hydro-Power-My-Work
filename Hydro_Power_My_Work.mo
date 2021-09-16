within ;
package Hydro_Power_My_Work
  package Tut_01
    model SimplePendulum "First simple version"
      constant Real g(unit="m/s2")=9.81 "Gravitationl constant";
      parameter Real L(unit="m")=1 "Length of the pendulum";
      Real Theta(unit="rad",start=0.1) "Angle of the pendulum";
      Real ThetaDot "Helping variable for 2nd derivative";
    equation
       ThetaDot = der(Theta);
       der(ThetaDot) = - g/L * sin(Theta);
      annotation (experiment(StopTime=10));
    end SimplePendulum;

    model SimplePendulumSIunits "Model using SI units"
      constant Modelica.SIunits.Acceleration g=9.81 "Gravitationl constant";
      parameter Modelica.SIunits.Length L=1 "Length of the pendulum";
      Modelica.SIunits.Angle Theta(start=0.1) "Angle of the pendulum";
      Real ThetaDot  "Helping variable for 2nd derivative";
    equation
       ThetaDot = der(Theta);
       der(ThetaDot) = - g/L * sin(Theta);
      annotation (experiment(StopTime=10));
    end SimplePendulumSIunits;

    model SimplePendulumUsingImports "Model using import"
      import SI = Modelica.SIunits;

      constant SI.Acceleration g=9.81 "Gravitationl constant";
      parameter SI.Length L=1 "Length of the pendulum";
      SI.Angle Theta(start=0.1) "Angle of the pendulum";
      Real ThetaDot  "Helping variable for 2nd derivative";
    equation
       ThetaDot = der(Theta);
       der(ThetaDot) = - g/L * sin(Theta);
      annotation (experiment(StopTime=10));
    end SimplePendulumUsingImports;
  end Tut_01;

  package Tut_02
  end Tut_02;
  annotation (uses(Modelica(version="3.2.3")));
end Hydro_Power_My_Work;
