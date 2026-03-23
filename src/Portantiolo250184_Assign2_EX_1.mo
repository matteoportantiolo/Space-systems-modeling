within ;
model Portantiolo250184_Assign2_EX_1

model Electromechanical_Block

  Modelica.Blocks.Interfaces.RealOutput theta annotation (Placement(
        transformation(extent={{86,20},{124,58}}),   iconTransformation(extent={{86,20},
              {124,58}})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={60,0})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(
    J=1/60,
    phi(start=-0.4*Modelica.Constants.pi, fixed=true),
    w(fixed=true, start=0)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={32,0})));
  Modelica.Electrical.Analog.Basic.RotationalEMF emf(k=0.3) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={4,0})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.1) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-56,42})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(i(fixed=true), L=0.001)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-22,42})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-84,0})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-50,-50},{-30,-30}})));
  Modelica.Blocks.Interfaces.RealInput voltage
    annotation (Placement(transformation(extent={{-128,-60},{-88,-20}})));
equation
  connect(angleSensor.phi, theta)
    annotation (Line(points={{71,0},{80,0},{80,39},{105,39}},
                                              color={0,0,127}));
  connect(angleSensor.flange, inertia.flange_b)
    annotation (Line(points={{50,0},{42,0}}, color={0,0,0}));
  connect(inertia.flange_a, emf.flange)
    annotation (Line(points={{22,0},{14,0}}, color={0,0,0}));
  connect(emf.p, inductor.n)
    annotation (Line(points={{4,10},{4,42},{-12,42}}, color={0,0,255}));
  connect(inductor.p, resistor.n)
    annotation (Line(points={{-32,42},{-46,42}}, color={0,0,255}));
  connect(resistor.p, signalVoltage.p)
    annotation (Line(points={{-66,42},{-84,42},{-84,10}}, color={0,0,255}));
  connect(emf.n, ground.p) annotation (Line(points={{4,-10},{4,-24},{-40,-24},{
          -40,-30}}, color={0,0,255}));
  connect(signalVoltage.n, ground.p) annotation (Line(points={{-84,-10},{-84,
          -24},{-40,-24},{-40,-30}}, color={0,0,255}));
  connect(signalVoltage.v, voltage) annotation (Line(points={{-72,0},{-64,0},{
            -64,-40},{-108,-40}},               color={0,0,127}));
    connect(theta, theta)
      annotation (Line(points={{105,39},{105,39}}, color={0,0,127}));
  annotation (
    uses(Modelica(version="4.0.0")),
    Diagram(coordinateSystem(extent={{-120,-100},{120,100}})),
    Icon(coordinateSystem(extent={{-120,-100},{120,100}})));
end Electromechanical_Block;

model Control_Block
  Modelica.Blocks.Interfaces.RealInput T1 annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-26,92})));
  Modelica.Blocks.Interfaces.RealInput theta
    annotation (Placement(transformation(extent={{-168,-40},{-128,0}})));
  Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold(threshold=
        294.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-32,38})));
  Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold1
    annotation (Placement(transformation(extent={{-96,-50},{-76,-30}})));
  Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold(threshold
      =-0.4*Modelica.Constants.pi)
    annotation (Placement(transformation(extent={{-96,-10},{-76,10}})));
  Modelica.Blocks.Logical.And and1
    annotation (Placement(transformation(extent={{-56,-28},{-36,-8}})));
  Modelica.Blocks.Logical.LogicalSwitch logicalSwitch
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{92,-10},{112,10}})));
  Modelica.Blocks.Sources.Constant const(k=0)
    annotation (Placement(transformation(extent={{32,-50},{52,-30}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={26,64})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 294.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={72,64})));
  Modelica.Blocks.Math.Add add(k1=-1, k2=+1)
    annotation (Placement(transformation(extent={{28,18},{48,38}})));
  Modelica.Blocks.Math.Gain gain(k=1*10^(-4))
    annotation (Placement(transformation(extent={{64,10},{84,30}})));
  Modelica.Blocks.Interfaces.RealOutput voltage annotation (Placement(
        transformation(extent={{144,-58},{180,-22}}),iconTransformation(extent={{144,-58},
              {180,-22}})));
equation
  connect(T1, lessEqualThreshold.u) annotation (Line(points={{-26,92},{-26,58},
            {-32,58},{-32,50}},
                              color={0,0,127}));
  connect(theta, lessEqualThreshold1.u) annotation (Line(points={{-148,-20},{
            -118,-20},{-118,-40},{-98,-40}},
                                           color={0,0,127}));
  connect(greaterEqualThreshold.u, theta) annotation (Line(points={{-98,0},{
            -118,0},{-118,-20},{-148,-20}},
                                          color={0,0,127}));
  connect(and1.u1, greaterEqualThreshold.y) annotation (Line(points={{-58,-18},
          {-68,-18},{-68,0},{-75,0}}, color={255,0,255}));
  connect(and1.u2, lessEqualThreshold1.y) annotation (Line(points={{-58,-26},{
          -68,-26},{-68,-40},{-75,-40}}, color={255,0,255}));
  connect(logicalSwitch.u1, and1.y) annotation (Line(points={{-12,8},{-26,8},{
          -26,-18},{-35,-18}}, color={255,0,255}));
  connect(logicalSwitch.u3, lessEqualThreshold.y)
    annotation (Line(points={{-12,-8},{-32,-8},{-32,27}}, color={255,0,255}));
  connect(switch1.u2, logicalSwitch.y)
    annotation (Line(points={{90,0},{11,0}},  color={255,0,255}));
  connect(switch1.u3, const.y) annotation (Line(points={{90,-8},{60,-8},{60,-40},
            {53,-40}},    color={0,0,127}));
  connect(logicalSwitch.u2, and1.y) annotation (Line(points={{-12,0},{-26,0},{
          -26,-18},{-35,-18}}, color={255,0,255}));
  connect(temperatureSensor.port, fixedTemperature.port)
    annotation (Line(points={{36,64},{62,64}}, color={191,0,0}));
  connect(gain.u, add.y) annotation (Line(points={{62,20},{56,20},{56,28},{49,
            28}},
                color={0,0,127}));
  connect(switch1.u1, gain.y) annotation (Line(points={{90,8},{90,12},{85,12},{
            85,20}},color={0,0,127}));
  connect(switch1.y, voltage)
    annotation (Line(points={{113,0},{114,0},{114,-40},{162,-40}},
                                               color={0,0,127}));
  connect(add.u2, T1) annotation (Line(points={{26,22},{-10,22},{-10,58},{-26,
            58},{-26,92}},
                     color={0,0,127}));
  connect(add.u1, temperatureSensor.T) annotation (Line(points={{26,34},{2,34},
            {2,64},{15,64}},color={0,0,127}));
    connect(voltage, voltage)
      annotation (Line(points={{162,-40},{162,-40}}, color={0,0,127}));
  annotation (
    uses(Modelica(version="4.0.0")),
    Diagram(coordinateSystem(extent={{-160,-100},{180,100}})),
    Icon(coordinateSystem(extent={{-160,-100},{180,100}})));
end Control_Block;

model Thermal_Block

block Variable_G_r
    Modelica.Blocks.Interfaces.RealInput theta
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,100}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,100})));
    Modelica.Blocks.Interfaces.RealOutput G_r
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,-100}),
                        iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,-100})));
    parameter Real eps_min( unit="-");
    parameter Real eps_max( unit="-");
    parameter Real A_r(  unit="-");
equation
  G_r = A_r*(eps_min + (((eps_max - eps_min)/(0.4*Modelica.Constants.pi))*(
    theta + 0.4*Modelica.Constants.pi)));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(extent=
             {{-66,76},{62,-80}}, lineColor={28,108,200}), Text(
          extent={{-50,24},{48,-24}},
          textColor={28,108,200},
          textString="G_r_VAR")}));
end Variable_G_r;

model Radiator_variable
extends Modelica.Thermal.HeatTransfer.Interfaces.Element1D;
  Modelica.Blocks.Interfaces.RealInput G_r
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=270,
        origin={2,98}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={2,98})));
equation
  Q_flow = G_r*Modelica.Constants.sigma*(port_a.T^4 - port_b.T^4);
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}), graphics={
        Rectangle(
          extent={{50,80},{90,-80}},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-90,80},{-50,-80}},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Line(points={{-36,10},{36,10}}, color={191,0,0}),
        Line(points={{-36,10},{-26,16}}, color={191,0,0}),
        Line(points={{-36,10},{-26,4}}, color={191,0,0}),
        Line(points={{-36,-10},{36,-10}}, color={191,0,0}),
        Line(points={{26,-16},{36,-10}}, color={191,0,0}),
        Line(points={{26,-4},{36,-10}}, color={191,0,0}),
        Line(points={{-36,-30},{36,-30}}, color={191,0,0}),
        Line(points={{-36,-30},{-26,-24}}, color={191,0,0}),
        Line(points={{-36,-30},{-26,-36}}, color={191,0,0}),
        Line(points={{-36,30},{36,30}}, color={191,0,0}),
        Line(points={{26,24},{36,30}}, color={191,0,0}),
        Line(points={{26,36},{36,30}}, color={191,0,0}),
        Text(
          extent={{-150,125},{150,85}},
          textString="%name",
          textColor={0,0,255}),
        Text(
          extent={{-150,-90},{150,-120}},
          textString="Gr=%Gr"),
        Rectangle(
          extent={{-50,80},{-44,-80}},
          lineColor={191,0,0},
          fillColor={191,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{45,80},{50,-80}},
          lineColor={191,0,0},
          fillColor={191,0,0},
          fillPattern=FillPattern.Solid)}),
    Documentation(info="<html>
<p>
This is a model describing the thermal radiation, i.e., electromagnetic
radiation emitted between two bodies as a result of their temperatures.
The following constitutive equation is used:
</p>
<blockquote><pre>
Q_flow = Gr*sigma*(port_a.T^4 - port_b.T^4);
</pre></blockquote>
<p>
where Gr is the radiation conductance and sigma is the Stefan-Boltzmann
constant (= Modelica.Constants.sigma). Gr may be determined by
measurements and is assumed to be constant over the range of operations.
</p>
<p>
For simple cases, Gr may be analytically computed. The analytical
equations use epsilon, the emission value of a body which is in the
range 0..1. Epsilon=1, if the body absorbs all radiation (= black body).
Epsilon=0, if the body reflects all radiation and does not absorb any.
</p>
<blockquote><pre>
Typical values for epsilon:
aluminium, polished    0.04
copper, polished       0.04
gold, polished         0.02
paper                  0.09
rubber                 0.95
silver, polished       0.02
wood                   0.85..0.9
</pre></blockquote>
<p><strong>Analytical Equations for Gr</strong></p>
<p>
<strong>Small convex object in large enclosure</strong>
(e.g., a hot machine in a room):
</p>
<blockquote><pre>
Gr = e*A
where
   e: Emission value of object (0..1)
   A: Surface area of object where radiation
      heat transfer takes place
</pre></blockquote>
<p><strong>Two parallel plates</strong>:</p>
<blockquote><pre>
Gr = A/(1/e1 + 1/e2 - 1)
where
   e1: Emission value of plate1 (0..1)
   e2: Emission value of plate2 (0..1)
   A : Area of plate1 (= area of plate2)
</pre></blockquote>
<p><strong>Two long cylinders in each other</strong>, where radiation takes
place from the inner to the outer cylinder):
</p>
<blockquote><pre>
Gr = 2*pi*r1*L/(1/e1 + (1/e2 - 1)*(r1/r2))
where
   pi: = Modelica.Constants.pi
   r1: Radius of inner cylinder
   r2: Radius of outer cylinder
   L : Length of the two cylinders
   e1: Emission value of inner cylinder (0..1)
   e2: Emission value of outer cylinder (0..1)
</pre></blockquote>
</html>"),
    uses(Modelica(version="4.0.0")));
end Radiator_variable;

  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Main_Body(C=1.5e5, T(
      start=298.15,
      displayUnit="K",
      fixed=true))
    annotation (Placement(transformation(extent={{-10,50},{10,70}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Panel1(C=1187.5, T(
      start=298.15,
      displayUnit="K",
      fixed=true))
    annotation (Placement(transformation(extent={{-90,10},{-70,30}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Radiator1(C=30, T(
      start=298.15,
      displayUnit="K",
      fixed=true))
    annotation (Placement(transformation(extent={{-90,-38},{-70,-18}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Radiator2(C=30, T(
      start=298.15,
      displayUnit="K",
      fixed=true))
    annotation (Placement(transformation(extent={{70,-38},{90,-18}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Panel2(C=1187.5, T(
      start=298.15,
      displayUnit="K",
      fixed=true))
    annotation (Placement(transformation(extent={{70,10},{90,30}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor Conductor12(G=10)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,0})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor Conductor14(G=10)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,-48})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor Conductor15(G=10)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={34,-48})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor Conductor13(G=10)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={32,0})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature DeepSpace(T(
        displayUnit="K") = 3)
    annotation (Placement(transformation(extent={{-186,76},{-166,96}})));
  Modelica.Thermal.HeatTransfer.Components.BodyRadiation Radiation2(Gr=0.35625)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-130,54})));
  Modelica.Thermal.HeatTransfer.Components.BodyRadiation Radiation3(Gr=0.35625)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-130,118})));
  Modelica.Thermal.HeatTransfer.Components.BodyRadiation Radiation1(Gr=1.575)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-130,86})));
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow Sun_Power2(Q_flow=500.175)
    annotation (Placement(transformation(extent={{-136,-10},{-116,10}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow Sun_Power3(Q_flow=500.175)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={116,0})));
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow Sun_Power1(Q_flow=202.5)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={32,42})));
  Radiator_variable radiator_variable annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-98,-72})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-44,86},{-24,106}})));
  Modelica.Blocks.Interfaces.RealOutput T1 annotation (Placement(transformation(
          extent={{110,66},{168,124}}), iconTransformation(extent={{110,66},{
              168,124}})));
  Modelica.Blocks.Interfaces.RealInput theta annotation (Placement(
        transformation(extent={{-202,-138},{-150,-86}}), iconTransformation(
          extent={{-202,-138},{-150,-86}})));
  Variable_G_r variable_G_r(
    eps_min=0.01,
    eps_max=0.98,
    A_r=0.25) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-120,-112})));
  Radiator_variable radiator_variable1 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={60,-90})));
equation
  connect(Panel1.port, Conductor12.port_a)
    annotation (Line(points={{-80,10},{-80,0},{-46,0}},      color={191,0,0}));
  connect(Panel2.port, Conductor13.port_a)
    annotation (Line(points={{80,10},{80,0},{42,0}},     color={191,0,0}));
  connect(Main_Body.port, Conductor12.port_b)
    annotation (Line(points={{0,50},{0,0},{-26,0}},     color={191,0,0}));
  connect(Conductor13.port_b, Conductor12.port_b)
    annotation (Line(points={{22,0},{-26,0}},     color={191,0,0}));
  connect(Radiator1.port, Conductor14.port_a)
    annotation (Line(points={{-80,-38},{-80,-48},{-46,-48}}, color={191,0,0}));
  connect(Radiator2.port, Conductor15.port_a)
    annotation (Line(points={{80,-38},{80,-48},{44,-48}}, color={191,0,0}));
  connect(Conductor15.port_b, Conductor12.port_b) annotation (Line(points={{24,-48},
            {0,-48},{0,0},{-26,0}},        color={191,0,0}));
  connect(Conductor14.port_b, Conductor12.port_b) annotation (Line(points={{-26,-48},
            {0,-48},{0,0},{-26,0}},        color={191,0,0}));
  connect(Main_Body.port, Radiation1.port_a) annotation (Line(points={{0,50},{0,
            42},{-68,42},{-68,86},{-120,86}},
                                            color={191,0,0}));
  connect(Panel2.port, Radiation3.port_a) annotation (Line(points={{80,10},{80,
            0},{60,0},{60,118},{-120,118}},   color={191,0,0}));
  connect(Radiation1.port_b, DeepSpace.port)
    annotation (Line(points={{-140,86},{-166,86}}, color={191,0,0}));
  connect(Radiation3.port_b, DeepSpace.port) annotation (Line(points={{-140,118},
            {-150,118},{-150,86},{-166,86}},
                                           color={191,0,0}));
  connect(Radiation2.port_b, DeepSpace.port) annotation (Line(points={{-140,54},
            {-150,54},{-150,86},{-166,86}},
                                          color={191,0,0}));
  connect(Sun_Power3.port, Conductor13.port_a)
    annotation (Line(points={{106,0},{42,0}},     color={191,0,0}));
  connect(Sun_Power1.port, Conductor12.port_b) annotation (Line(points={{22,42},
            {0,42},{0,0},{-26,0}},   color={191,0,0}));
  connect(Panel1.port, Radiation2.port_a) annotation (Line(points={{-80,10},{
            -80,0},{-96,0},{-96,54},{-120,54}},   color={191,0,0}));
  connect(Sun_Power2.port, Radiation2.port_a) annotation (Line(points={{-116,0},
            {-96,0},{-96,54},{-120,54}},      color={191,0,0}));
  connect(radiator_variable.port_a, Conductor14.port_a) annotation (Line(points={{-88,-72},
            {-56,-72},{-56,-48},{-46,-48}},         color={191,0,0}));
  connect(radiator_variable.port_b, DeepSpace.port) annotation (Line(points={{-108,
            -72},{-150,-72},{-150,86},{-166,86}},    color={191,0,0}));
  connect(temperatureSensor.port, Radiation1.port_a) annotation (Line(points={{-44,96},
            {-110,96},{-110,86},{-120,86}},       color={191,0,0}));
  connect(temperatureSensor.T, T1) annotation (Line(points={{-23,96},{-22,95},{
            139,95}},          color={0,0,127}));
  connect(theta, variable_G_r.theta) annotation (Line(points={{-176,-112},{-130,
            -112}},                       color={0,0,127}));
  connect(variable_G_r.G_r, radiator_variable.G_r) annotation (Line(points={{-110,
            -112},{-98.2,-112},{-98.2,-81.8}},      color={0,0,127}));
  connect(radiator_variable1.port_a, Conductor15.port_a) annotation (Line(
        points={{70,-90},{80,-90},{80,-48},{44,-48}},   color={191,0,0}));
  connect(radiator_variable1.port_b, DeepSpace.port) annotation (Line(points={{50,-90},
            {-150,-90},{-150,86},{-166,86}},         color={191,0,0}));
  connect(radiator_variable1.G_r, variable_G_r.G_r) annotation (Line(points={{59.8,
            -99.8},{59.8,-112},{-110,-112}},     color={0,0,127}));
  annotation (
    uses(Modelica(version="4.0.0")),
    Diagram(coordinateSystem(extent={{-200,-140},{160,140}})),
    Icon(coordinateSystem(extent={{-200,-140},{160,140}})));
end Thermal_Block;


  Thermal_Block thermal_Block
    annotation (Placement(transformation(extent={{-28,34},{30,74}})));
  Electromechanical_Block electromechanical_Block
    annotation (Placement(transformation(extent={{-84,-68},{-26,-20}})));
  Control_Block control_Block
    annotation (Placement(transformation(extent={{16,-68},{82,-20}})));
equation
  connect(control_Block.voltage, electromechanical_Block.voltage) annotation (
      Line(points={{78.5059,-53.6},{96,-53.6},{96,-80},{-96,-80},{-96,-53.6},{
          -81.1,-53.6}},    color={0,0,127}));
  connect(control_Block.theta, electromechanical_Block.theta) annotation (Line(
        points={{18.3294,-48.8},{-12,-48.8},{-12,-34.64},{-29.625,-34.64}},
                                                                        color={
          0,0,127}));
  connect(electromechanical_Block.theta, thermal_Block.theta) annotation (Line(
        points={{-29.625,-34.64},{-12,-34.64},{-12,0},{-40,0},{-40,38},{
          -24.1333,38}},
        color={0,0,127}));
  connect(thermal_Block.T1, control_Block.T1) annotation (Line(points={{26.6167,
          67.5714},{26.6167,68},{42.0118,68},{42.0118,-21.92}},
                                                          color={0,0,127}));
end Portantiolo250184_Assign2_EX_1;
