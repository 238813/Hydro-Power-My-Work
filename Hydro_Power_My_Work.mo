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
    model Motor
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-56,-32},{-36,-12}})));
      Modelica.Electrical.Analog.Basic.Resistor resistor
        annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor
        annotation (Placement(transformation(extent={{-26,30},{-6,50}})));
      Modelica.Electrical.Analog.Basic.EMF emf
        annotation (Placement(transformation(extent={{10,0},{30,20}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-76,10})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia
        annotation (Placement(transformation(extent={{40,0},{60,20}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
        "Flange of right shaft"
        annotation (Placement(transformation(extent={{76,0},{96,20}})));
      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-140,-10},{-100,30}})));
    equation
      connect(emf.flange, inertia.flange_a)
        annotation (Line(points={{30,10},{40,10}}, color={0,0,0}));
      connect(inertia.flange_b, flange)
        annotation (Line(points={{60,10},{86,10}}, color={0,0,0}));
      connect(signalVoltage.p, resistor.p) annotation (Line(points={{-76,20},{
              -76,40},{-60,40}}, color={0,0,255}));
      connect(resistor.n, inductor.p)
        annotation (Line(points={{-40,40},{-26,40}}, color={0,0,255}));
      connect(inductor.n, emf.p)
        annotation (Line(points={{-6,40},{20,40},{20,20}}, color={0,0,255}));
      connect(signalVoltage.n, emf.n)
        annotation (Line(points={{-76,0},{20,0}}, color={0,0,255}));
      connect(ground.p, emf.n)
        annotation (Line(points={{-46,-12},{-46,0},{20,0}}, color={0,0,255}));
      connect(signalVoltage.v, u)
        annotation (Line(points={{-88,10},{-120,10}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Motor;

    package MotorDrive

    end MotorDrive;
  end Tut_02;
  annotation (uses(Modelica(version="3.2.3")));
end Hydro_Power_My_Work;
