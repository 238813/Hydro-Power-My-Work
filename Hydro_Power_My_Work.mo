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

    model Motor_Drive
      Motor motor(
        resistor(R=0.5),
        inductor(L=0.05),
        inertia(J=0.001))
        annotation (Placement(transformation(extent={{-4,-10},{16,10}})));
      Modelica.Blocks.Math.Feedback feedback
        annotation (Placement(transformation(extent={{-64,-10},{-44,10}})));
      Modelica.Blocks.Continuous.PID PID
        annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
      Modelica.Blocks.Sources.Step step(startTime=0.5)
        annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
      Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
        annotation (Placement(transformation(extent={{26,-10},{46,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
        annotation (Placement(transformation(extent={{52,-10},{72,10}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation
        (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={54,-38})));
    equation
      connect(feedback.y, PID.u)
        annotation (Line(points={{-45,0},{-40,0}}, color={0,0,127}));
      connect(motor.u, PID.y) annotation (Line(points={{-6,1},{-10,1},{-10,0},{
              -17,0}}, color={0,0,127}));
      connect(feedback.u1, step.y)
        annotation (Line(points={{-62,0},{-67,0}}, color={0,0,127}));
      connect(idealGear.flange_a, motor.flange) annotation (Line(points={{26,0},
              {24,0},{24,1},{14.6,1}}, color={0,0,0}));
      connect(idealGear.flange_b, inertia.flange_a)
        annotation (Line(points={{46,0},{52,0}}, color={0,0,0}));
      connect(inertia.flange_b, angleSensor.flange)
        annotation (Line(points={{72,0},{72,-38},{64,-38}}, color={0,0,0}));
      connect(angleSensor.phi, feedback.u2) annotation (Line(points={{43,-38},{
              -54,-38},{-54,-8}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Motor_Drive;
  end Tut_02;
  annotation (uses(Modelica(version="3.2.3")));
end Hydro_Power_My_Work;
