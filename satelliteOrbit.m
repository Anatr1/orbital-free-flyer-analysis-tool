
function satelliteOrbit()
    % Parametri dell'orbita
    raggioOrbita = 50000;  % Raggio dell'orbita circolare
    velocitaIniziale = 0.05;  % Velocità iniziale del satellite
    deltaVelocita = 0.01;  % Incremento/Decremento della velocità
    
    % Inizializzazione delle variabili
    theta = 0;  % Angolo iniziale
    velocita = velocitaIniziale;  % Velocità corrente del satellite
    pausaSimulazione = false;  % Flag per la pausa della simulazione
    fineSimulazione = false;  % Flag per l'uscita dalla simulazione
    
    % Creazione della figura
    figure('KeyPressFcn', @keyPressCallback);
    axis([-raggioOrbita*2 raggioOrbita*2 -raggioOrbita*2 raggioOrbita*2]);
    hold on;
    hSatellite = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    grid on
    hOrbita = viscircles([0, 0], raggioOrbita, 'LineStyle', '--');
    hold off;
    
    % Ciclo principale
    while ~fineSimulazione
        while pausaSimulazione
            pause(0.5);
        end
        
        % Calcolo le coordinate del satellite
        x = raggioOrbita * cos(theta);
        y = raggioOrbita * sin(theta);
        
        % Aggiorno la posizione del satellite
        set(hSatellite, 'XData', x, 'YData', y);
        
        % Aggiorno l'angolo
        theta = theta + velocita;
        
        % Controllo se l'angolo supera 2*pi (completato un giro)
        if theta >= 2*pi
            theta = 0;
        end
        
        drawnow;
    end
    
    % Funzione di callback per i tasti premuti
    function keyPressCallback(~, event)
        if strcmp(event.Key, 'space')  % Pausa/riprendi simulazione
            pausaSimulazione = ~pausaSimulazione;
        elseif strcmp(event.Key, 'escape')  % Esci dalla simulazione
            fineSimulazione = true;
        elseif strcmp(event.Key, 'uparrow')  % Aumenta la velocità
            velocita = velocita + deltaVelocita;
        elseif strcmp(event.Key, 'downarrow')  % Diminuisci la velocità
            velocita = velocita - deltaVelocita;
        end
    end
end
