%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% active_set.m
%
% Structure for the active set of contacts






classdef active_set < handle
  properties (SetAccess='public')
    psi             % gap distanct
    BIDX_vertex     % body index of vertex
    vertexIDX       % vertex index
    BIDX_face       % body index of face
    faceIDX         % face index
  end
  
  methods
    %%%%%%%%%%%%%%%
    % Constructor %
    %%%%%%%%%%%%%%%
    function as = active_set(psi, BIDX_vertex, vertexIDX, BIDX_face, faceIDX)
      as.psi = psi;
      as.BIDX_vertex = BIDX_vertex;
      as.vertexIDX = vertexIDX;
      as.BIDX_face = BIDX_face;
      as.faceIDX = faceIDX;
    end
    
  end
end




