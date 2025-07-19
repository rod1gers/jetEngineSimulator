package JetEngine
   
connector FluidPort
  Real p "Pressure [Pa]";
  flow Real m_flow "Mass Flow Rate [kg/s]";
end FluidPort;

  model APU
    FluidPort outPort;
    parameter Real pressure = 3e5;
    parameter Real m_flow_nominal = 0.5;
  equation
    outPort.p = pressure;
    outPort.m_flow = m_flow_nominal;
  end APU;

  model StarterValve
    FluidPort inPort, outPort;
    input Real opening(start=0);
    parameter Real Kv = 0.01;
  equation
    outPort.p = inPort.p - Kv * opening * abs(inPort.m_flow);
    // Note: mass flow conservation is handled by connect() statements
  end StarterValve;

  connector Shaft
    Real w "Angular Velocity";
    flow Real tau "Torque";
  end Shaft;

  model StarterTurbine
    FluidPort inPort, outPort;
    Shaft shaft;
    parameter Real efficiency = 0.9;
    parameter Real k = 5; // flow-to-torque coefficient
  equation
    inPort.p - outPort.p = k * abs(inPort.m_flow); // Pressure drop with abs()
    // Note: mass flow conservation is handled by connect() statements
    shaft.tau = efficiency * k * abs(inPort.m_flow); // Use abs() for torque
  end StarterTurbine;

  model EngineShaft
    Shaft shaft;
    parameter Real J = 10; // inertia
    Modelica.Units.SI.AngularVelocity w(start=0);
    
  equation
    der(w) = shaft.tau / J;
    shaft.w = w;
  end EngineShaft;

  model StarterSystem
    APU apu;
    StarterValve valve;
    StarterTurbine turbine;
    EngineShaft engineShaft;
    
    Modelica.Blocks.Sources.Step valveCmd(startTime=1, height=1);
    
  initial equation
    // Set initial conditions to avoid warnings
    engineShaft.w = 0.0;  // Start with zero angular velocity
    
  equation
    connect(apu.outPort, valve.inPort);
    connect(valve.outPort, turbine.inPort);
    connect(turbine.shaft, engineShaft.shaft);
    
    // Connect the valve command to the valve opening
    valve.opening = valveCmd.y;
    
    // Add outlet boundary condition - fluid must go somewhere
    turbine.outPort.p = 1e5; // Atmospheric pressure outlet
    
  end StarterSystem;
end JetEngine;
