

function simulateSatelliteOrbit()
    % Inizializzazione delle variabili
    radius = 10; % Raggio dell'orbita circolare
    satellitePos = [radius; 0; 0]; % Posizione iniziale del satellite
    satelliteVel = [0; radius; 0]; % Velocità iniziale del satellite
    timeStep = 0.5; % Passo di tempo della simulazione
    
    % Creazione della figura
    figure('KeyPressFcn', @keyPressCallback);
    hold on;
    view(3);
    axis([-radius*2, radius*2, -radius*2, radius*2, -radius*2, radius*2]);
    grid on;
    
    % Disegno dell'orbita circolare[
    theta = linspace(0, 2*pi, 100);
    orbitX = radius * cos(theta);
    orbitY = radius * sin(theta);
    orbitZ = zeros(size(theta));
    plot3(orbitX, orbitY, orbitZ, 'b--');
    
    % Disegno del satellite come punto
    satellitePlot = plot3(satellitePos(1), satellitePos(2), satellitePos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % Variabili di controllo della simulazione
    isPaused = false;
    isRunning = true;
    
    % Ciclo principale della simulazione
    while isRunning
        % Calcolo dell'accelerazione
        acceleration = calculateAcceleration(satellitePos, satelliteVel);
        
        % Aggiornamento della posizione e della velocità del satellite
        satellitePos = satellitePos + satelliteVel * timeStep;
        satelliteVel = satelliteVel + acceleration * timeStep;
        
        % Aggiornamento della posizione del satellite nel grafico
        set(satellitePlot, 'XData', satellitePos(1), 'YData', satellitePos(2), 'ZData', satellitePos(3));
        drawnow;
        
        % Controllo della pausa o dell'interruzione della simulazione
        while isPaused
            pause(0.1);
        end
        
        if ~isRunning
            break;
        end
    end
    
    % Funzione di calcolo dell'accelerazione
    function acceleration = calculateAcceleration(position, velocity)
        % Richiesta di input per le variazioni di velocità
        disp('Inserisci le variazioni di velocità:');
        disp('Formato: [variazione_X, variazione_Y, variazione_Z]');
        disp('Esempio: [0, 0, 0] per nessuna variazione');
        disp('Premi "p" per mettere in pausa o riprendere la simulazione');
        disp('Premi "q" per uscire dalla simulazione');
        inputString = input('>>> ', 's');
        
        if strcmpi(inputString, 'p')
            isPaused = ~isPaused;
            acceleration = [0; 0; 0];
            return;
        elseif strcmpi(inputString, 'q')
            isRunning = false;
            acceleration = [0; 0; 0];
            return;
        else
            variations = str2num(inputString); %#ok<ST2NM>
        end
        
        acceleration = variations(:);
    end

    % Funzione di callback per la gestione degli input da tastiera
    function keyPressCallback(~, event)
        if strcmpi(event.Key, 'p')
            isPaused = ~isPaused;
        elseif strcmpi(event.Key, 'q')
            isRunning = false;
        end
    end
end
