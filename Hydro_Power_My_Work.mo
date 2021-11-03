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
      Modelica.Blocks.Continuous.PID PID(k=10)
        annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
      Modelica.Blocks.Sources.Step step(startTime=0.5)
        annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
      Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
        annotation (Placement(transformation(extent={{26,-10},{46,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
        annotation (Placement(transformation(extent={{52,-10},{72,10}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
         Placement(transformation(
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
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(extent={{-4,10},{16,-10}}, lineColor={238,46,47}),
            Rectangle(extent={{-4,10},{16,-10}}, lineColor={229,44,47}),
            Rectangle(
              extent={{-4,10},{16,-10}},
              lineColor={229,44,47},
              fillColor={0,0,0},
              fillPattern=FillPattern.None),
            Bitmap(extent={{-4,10},{16,-10}}, fileName=""),
            Bitmap(extent={{-4,10},{16,-10}}, fileName=""),
            Bitmap(extent={{-4,10},{16,-12}}, fileName=""),
            Polygon(
              points={{0,-10},{-4,-20},{14,-20},{12,-10},{0,-12},{4,-10},{0,-10}},
              lineColor={229,44,47},
              fillColor={255,0,0},
              fillPattern=FillPattern.Solid,
              lineThickness=1),
            Line(
              points={{-4,10},{-2,6},{-2,-10},{16,-10},{16,10},{-4,10},{-4,-10}},
              color={229,44,47},
              thickness=1),
            Bitmap(extent={{-4,10},{16,-10}}, fileName="")}),
        experiment(StopTime=100));
    end Motor_Drive;
  end Tut_02;

  package Tut_03
    package Components
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

      model DCMachine
        parameter Modelica.SIunits.Resistance R=0.5
                                                   "Resistance of Armature";
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-26,-32},{-6,-12}})));
        Modelica.Electrical.Analog.Basic.Resistor resistor
          annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor
          annotation (Placement(transformation(extent={{-26,30},{-6,50}})));
        Modelica.Electrical.Analog.Basic.EMF emf
          annotation (Placement(transformation(extent={{10,0},{30,20}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{40,0},{60,20}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
          "Flange of right shaft"
          annotation (Placement(transformation(extent={{76,0},{96,20}})));
        Modelica.Electrical.Analog.Interfaces.PositivePin P "Positive electrical pin"
          annotation (Placement(transformation(extent={{-110,72},{-90,92}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin N "Negative electrical pin"
          annotation (Placement(transformation(extent={{-112,-72},{-92,-52}}),
              iconTransformation(extent={{-112,-72},{-92,-52}})));
        parameter Modelica.SIunits.Inductance L=0.1 "Inductance of DC Machine";
      equation
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{30,10},{40,10}}, color={0,0,0}));
        connect(inertia.flange_b, flange)
          annotation (Line(points={{60,10},{86,10}}, color={0,0,0}));
        connect(resistor.n, inductor.p)
          annotation (Line(points={{-40,40},{-26,40}}, color={0,0,255}));
        connect(inductor.n, emf.p)
          annotation (Line(points={{-6,40},{20,40},{20,20}}, color={0,0,255}));
        connect(resistor.p, P) annotation (Line(points={{-60,40},{-82,40},{-82,82},{-100,
                82}}, color={0,0,255}));
        connect(emf.n, N) annotation (Line(points={{20,0},{-40,0},{-40,-62},{-102,-62}},
              color={0,0,255}));
        connect(emf.n, ground.p)
          annotation (Line(points={{20,0},{-16,0},{-16,-12}}, color={0,0,255}));
        connect(N, N) annotation (Line(points={{-102,-62},{-102,-77},{-102,-77},{-102,
                -62}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Bitmap(extent={{92,-44},{-92,74}}, fileName="modelica://Hydro_Power_My_Work/../FM3217/Resources/Images/dc-motor.jpg")}),
                                                                       Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          Documentation(info="<html>
<p><img src=\"modelica://Hydro_Power_My_Work/../FM3217/Resources/Images/dc-motor.jpg\"/></p>
<h4>This is a simple DC Machine&apos;s Image from Professor.</h4>
</html>"));
      end DCMachine;

      model Rload "Resistive-load"
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R_load)
          annotation (Placement(transformation(
              extent={{14,-14},{-14,14}},
              rotation=90,
              origin={0,0})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p1
          "Positive electrical pin"
          annotation (Placement(transformation(extent={{-10,94},{10,114}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n1
                      "Negative electrical pin"
          annotation (Placement(transformation(extent={{-10,-114},{10,-94}})));
        parameter Modelica.SIunits.Resistance R_load=0.5;
      equation
        connect(resistor.p, p1) annotation (Line(points={{-8.88178e-16,14},{0,
                14},{0,104}}, color={0,0,255}));
        connect(resistor.n, n1) annotation (Line(points={{-2.66454e-15,-14},{
                -2.66454e-15,-60},{0,-60},{0,-104}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Rload;

      model RLload
        extends Rload;
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,-2})));
        parameter Modelica.SIunits.Inductance L_load=0.1 "Load Inductance";
      equation
        connect(inductor.p, p1) annotation (Line(points={{-40,8},{-40,40},{0,40},
                {0,104}}, color={0,0,255}));
        connect(inductor.n, n1) annotation (Line(points={{-40,-12},{-40,-40},{0,
                -40},{0,-104}}, color={0,0,255}));
      end RLload;

      model RLC_load "RLC load"
        extends RLload;
        parameter Modelica.SIunits.Capacitance C_load=0.1 "Load of capacitor";
        Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=C_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,0})));
      equation
        connect(capacitor.p, p1) annotation (Line(points={{-80,10},{-80,40},{0,
                40},{0,104}}, color={0,0,255}));
        connect(capacitor.n, n1) annotation (Line(points={{-80,-10},{-80,-40},{
                0,-40},{0,-104}}, color={0,0,255}));
      end RLC_load;

      model Turbine
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J_t)
          annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
            tau_constant=T_t)
          annotation (Placement(transformation(extent={{88,-10},{68,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
            Placement(transformation(rotation=0, extent={{-110,-10},{-90,10}})));
        parameter Modelica.SIunits.Inertia J_t=2 "Turbine Inertia";
        parameter Modelica.SIunits.AngularMomentum T_t=10 "Turbine torque";
      equation
        connect(inertia.flange_b, constantTorque.flange)
          annotation (Line(points={{12,0},{68,0}}, color={0,0,0}));
        connect(flange_a, inertia.flange_a)
          annotation (Line(points={{-100,0},{-8,0}}, color={0,0,0}));
        annotation (Icon(graphics={Bitmap(extent={{-112,-98},{52,90}}, fileName
                  ="modelica://Hydro_Power_My_Work/../FM3217/Resources/Images/Turbine.png")}));
      end Turbine;
    end Components;

    package Tests
      model Motor_Drive
        Tut_02.Motor motor(
          resistor(R=0.5),
          inductor(L=0.05),
          inertia(J=0.001))
          annotation (Placement(transformation(extent={{-4,-10},{16,10}})));
        Modelica.Blocks.Math.Feedback feedback
          annotation (Placement(transformation(extent={{-64,-10},{-44,10}})));
        Modelica.Blocks.Continuous.PID PID(k=10)
          annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
        Modelica.Blocks.Sources.Step step(startTime=0.5)
          annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
        Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
          annotation (Placement(transformation(extent={{26,-10},{46,10}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
          annotation (Placement(transformation(extent={{52,-10},{72,10}})));
        Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
           Placement(transformation(
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
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(extent={{-4,10},{16,-10}}, lineColor={238,46,47}),
              Rectangle(extent={{-4,10},{16,-10}}, lineColor={229,44,47}),
              Rectangle(
                extent={{-4,10},{16,-10}},
                lineColor={229,44,47},
                fillColor={0,0,0},
                fillPattern=FillPattern.None),
              Bitmap(extent={{-4,10},{16,-10}}, fileName=""),
              Bitmap(extent={{-4,10},{16,-10}}, fileName=""),
              Bitmap(extent={{-4,10},{16,-12}}, fileName=""),
              Polygon(
                points={{0,-10},{-4,-20},{14,-20},{12,-10},{0,-12},{4,-10},{0,-10}},
                lineColor={229,44,47},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid,
                lineThickness=1),
              Line(
                points={{-4,10},{-2,6},{-2,-10},{16,-10},{16,10},{-4,10},{-4,-10}},
                color={229,44,47},
                thickness=1),
              Bitmap(extent={{-4,10},{16,-10}}, fileName="")}),
          experiment(StopTime=100));
      end Motor_Drive;

      model DCMachineTest
        Components.DCMachine DCMachine
          annotation (Placement(transformation(extent={{-32,-16},{-2,14}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(
              extent={{9,-9},{-9,9}},
              rotation=90,
              origin={-53,1})));
        Modelica.Blocks.Sources.Step step
          annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{4,-10},{24,10}})));
      equation
        connect(signalVoltage.p, DCMachine.P) annotation (Line(points={{-53,10},
                {-32,10},{-32,11.3}}, color={0,0,255}));
        connect(signalVoltage.n, DCMachine.N) annotation (Line(points={{-53,-8},
                {-52,-8},{-52,-10.3},{-32.3,-10.3}}, color={0,0,255}));
        connect(step.y, signalVoltage.v) annotation (Line(points={{-77,0},{-70,
                0},{-70,1},{-63.8,1}}, color={0,0,127}));
        connect(inertia.flange_a, DCMachine.flange)
          annotation (Line(points={{4,0},{4,0.5},{-4.1,0.5}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCMachineTest;

      model DCGeneratorTest
        Components.DCMachine DCMachine1
          annotation (Placement(transformation(extent={{-48,-30},{48,26}})));
        parameter Modelica.SIunits.Inductance L_load=0.1;
        Components.RLC_load rLC_load
          annotation (Placement(transformation(extent={{-92,-14},{-42,14}})));
        Components.Turbine turbine annotation (Placement(transformation(
                rotation=0, extent={{56,-28},{114,30}})));
      equation
        connect(DCMachine1.flange, turbine.flange_a) annotation (Line(points={{
                41.28,0.8},{48.95,0.8},{48.95,1},{56,1}}, color={0,0,0}));
        connect(rLC_load.p1, DCMachine1.P) annotation (Line(points={{-67,14.56},
                {-67,20.96},{-48,20.96}}, color={0,0,255}));
        connect(rLC_load.n1, DCMachine1.N) annotation (Line(points={{-67,-14.56},
                {-67,-19.36},{-48.96,-19.36}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCGeneratorTest;
    end Tests;
  end Tut_03;

  package Tut_4
    model ElectricKettle
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=26.45, useHeatPort=
            true) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={14,2})));
      Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(V=sqrt(2)*230,
          freqHz=50) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-94,2})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-104,-60},{-84,-40}})));
      Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
        annotation (Placement(transformation(extent={{-46,12},{-26,32}})));
      Modelica.Blocks.Math.Mean mean(f=50) annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-46,-4})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Water(C=4.18e3*1.7,
          T(start=283.15))
        annotation (Placement(transformation(extent={{26,28},{46,48}})));
      Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor temperatureSensor
        annotation (Placement(transformation(extent={{48,-8},{68,12}})));
      Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch
        annotation (Placement(transformation(extent={{-90,12},{-70,32}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor KattleWall(G=5)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={38,-42})));
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature roomTemperature(T=
            294.15) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={70,-76})));
      Modelica.Blocks.Logical.OnOffController onOffController(bandwidth=3)
        annotation (Placement(transformation(extent={{70,-2},{90,18}})));
      Modelica.Blocks.Sources.Constant const(k=95)
        annotation (Placement(transformation(extent={{30,64},{50,84}})));
    equation
      connect(sineVoltage.n, resistor.n) annotation (Line(points={{-94,-8},{-94,
              -18},{14,-18},{14,-8}}, color={0,0,255}));
      connect(ground.p, resistor.n) annotation (Line(points={{-94,-40},{-94,-18},
              {14,-18},{14,-8}}, color={0,0,255}));
      connect(powerSensor.nc, resistor.p)
        annotation (Line(points={{-26,22},{14,22},{14,12}}, color={0,0,255}));
      connect(powerSensor.pv, resistor.p)
        annotation (Line(points={{-36,32},{14,32},{14,12}}, color={0,0,255}));
      connect(powerSensor.nv, resistor.n) annotation (Line(points={{-36,12},{
              -36,-18},{14,-18},{14,-8}}, color={0,0,255}));
      connect(powerSensor.power, mean.u)
        annotation (Line(points={{-46,11},{-46,3.2}}, color={0,0,127}));
      connect(resistor.heatPort, Water.port)
        annotation (Line(points={{24,2},{36,2},{36,28}}, color={191,0,0}));
      connect(temperatureSensor.port, Water.port)
        annotation (Line(points={{48,2},{36,2},{36,28}}, color={191,0,0}));
      connect(sineVoltage.p, switch.p) annotation (Line(points={{-94,12},{-94,
              22},{-90,22}}, color={0,0,255}));
      connect(switch.n, powerSensor.pc)
        annotation (Line(points={{-70,22},{-46,22}}, color={0,0,255}));
      connect(KattleWall.port_a, Water.port) annotation (Line(points={{38,-32},
              {38,2},{36,2},{36,28}}, color={191,0,0}));
      connect(roomTemperature.port, KattleWall.port_b) annotation (Line(points=
              {{60,-76},{38,-76},{38,-52}}, color={191,0,0}));
      connect(temperatureSensor.T, onOffController.u)
        annotation (Line(points={{68,2},{68,2}}, color={0,0,127}));
      connect(const.y, onOffController.reference) annotation (Line(points={{51,
              74},{66,74},{66,14},{68,14}}, color={0,0,127}));
      connect(onOffController.y, switch.control) annotation (Line(points={{91,8},
              {92,8},{92,92},{-80,92},{-80,34}}, color={255,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<p>Electric Kettle</p>
<ul>
<li>Volume of 1.7 litre</li>
<li>230V mains supply</li>
<li>Power of 2000 Watt</li>
</ul>
<p><br>Question: What should the resistance of the heating resistor be?</p>
<p><br>Power= Voltage * Current</p>
<p>Resistance= Voltage/Current</p>
<p><br>This leads to:</p>
<p><br>For a 2000 Watt consuming resistor the resistance should be:</p>
<p>R=V/I=V/(P/V)=V^2/P</p>
<h4>Heat capacity of water</h4>
<p>1 calory= heat energy needed to heat up one gram of water by 1 Kelvin.</p>
<p>1 calory= 4.18 J/(g k)</p>
</html>"),
        experiment(StopTime=6000, __Dymola_NumberOfIntervals=5000));
    end ElectricKettle;
  end Tut_4;
  annotation (uses(Modelica(version="3.2.3")));
end Hydro_Power_My_Work;
