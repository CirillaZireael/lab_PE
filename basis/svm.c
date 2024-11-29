/************************************/
/* C-Functionen, eingebunden in Corpus s_svm.c */
/************************************/

#include "svm.h"
// svm.h enhtält
//      - Constanten
//      - Einbindung weiterer Standardfunktionen (math.h...)
//      - Initialisierung der Funktionen


//  ----------------------- Variablen ------------------------------------
float f_takt, f_sw, modgrad, alpha, sektor; //f_takt: Taktfrequenz, f_sw: Schaltfrequenz, modgrad: Modulationsgrad, 
                                            //alpha: Winkel im Sektor, sektor: Sektor des RZ
int pulse[3]; //enthät die Ausgangssignale, wobei -1: unteres Ventil offen, 1: oberes Ventil offen
float T_L, T_R, T_0; //T_L: Einschaltzeit des linken RZ, T_R: Einschaltzeit des rechten RZ, T_0: Einschaltzeit des Nullraumzeigers
float t_zeitpunkt, N, T_first, T_second; //t_zeitpunkt: Zeitpunkt innerhalb der Schaltperiode, N: Anzahl der Schritte innerhalb der Schaltperiode
                                         //T_first:  Einschaltzeit des zuerst einzuschaltenden RZ, T_second: Einschaltzeit des letzten RZ
int Z[8][3]={{-1, -1, -1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,1,1},{-1,-1,1},{1,-1,1},{1,1,1}};
                                         //Matrix enthält die 8 möglichen RZ
int Z_first,Z_second;                    //Z_first: zuerst einzuschaltender RZ, Z_second: zuletzt einzuschaltender RZ
        
//  ----------------------------------------------------------------------

// Initialisierungen
// wird automatisch beim Simulationsstart aufgerufen.
// Parameter:
void init_var_svm(float f_takt1, float f_sw1)
{
    // alte Zustände zurücksetzen
    f_takt = f_takt1; 
    f_sw = f_sw1;
    N = f_takt / f_sw;
    t_zeitpunkt = 0;
    pulse[0]=0;
    pulse[1]=0;
    pulse[2]=0;
}

//  ----------------------------------------------------------------------

// SVM
// Paramter: - keine

void svm(float u_dc, float u_soll_betrag, float gamma, double *out_svm){
    
    if(t_zeitpunkt > N) t_zeitpunkt = 0;        //wenn Taktzeit überschritten wird: Reset
    //Bestimmung des Modulationsgrades
        modgrad = 2*u_soll_betrag / u_dc;
    //Bestimmung des Sektors und des Winkels im Sektor
        alpha  = gamma/PI*3;
        sektor = floor(alpha);
        alpha  = (alpha-(float)sektor)*PI/3;
        sektor = sektor+1;
    //Bestimmung der Einschaltzeiten des linken und rechten Raumzeigers sowie des Nullraumzeigers
        T_L =  modgrad * sin(alpha)*N;
        T_R =  modgrad * sin(PI/3-alpha)*N;
        T_0 = N - T_L - T_R;
     //Festlegung des zuerst einzuschaltenden RZ
        if(sektor == 1 || sektor == 3 || sektor ==5)    //in den ungeraden Sektoren wird zuerst der rechte RZ eingeschaltet
        {Z_first = sektor;
        Z_second = sektor + 1;
        T_first  = T_R;
        T_second = T_L;}
        else                                            //in den geraden Sektoren wird zuerst der linke RZ geschaltet
        {Z_first = sektor + 1;
        Z_second = sektor;
        T_first  = T_L;
        T_second = T_R;}
        if(Z_first ==7) Z_first=1;
        if(Z_second == 7)Z_second=1;
     //Schalten der Raumzeiger
        
        //für die Zeit T_0/4 wird der RZ Z0 geschaltet
        if(t_zeitpunkt<=T_0/4) 
        {pulse[0] = Z[0][0]; pulse[1] = Z[0][1]; pulse[2]=Z[0][2];}
        
        //für die Zeit T_0/4 bis T_0/4 + T_first/2 wird der RZ Z_first geschaltet
        if(t_zeitpunkt>T_0/4 && t_zeitpunkt<= (T_0/4 + T_first/2))
        {pulse[0] = Z[Z_first][0]; pulse[1] = Z[Z_first][1]; pulse[2]=Z[Z_first][2];} 
        
        //für die Zeit T_0/4 + T_first/2 bis T_0/4 + T_first/2 + T_second/2 wird der RZ Z_second eingeschaltet
        if(t_zeitpunkt>(T_0/4+T_first/2) && t_zeitpunkt<= (T_0/4 + T_first/2+T_second/2))
        {pulse[0] = Z[Z_second][0]; pulse[1] = Z[Z_second][1]; pulse[2]=Z[Z_second][2];}
        
        //für die Zeit T_0/4 + T_first/2 + T_second/2 bis 3/4*T_0 + T_first/2 + T_second/2 wird der RZ Z7 eingeschaltet
        if(t_zeitpunkt>(T_0/4+T_first/2+T_second/2) && t_zeitpunkt<= (3*T_0/4 + T_first/2+T_second/2))
        {pulse[0] = Z[7][0]; pulse[1] = Z[7][1]; pulse[2]=Z[7][2];}
        
        //für die Zeit 3/4*T_0 + T_first/2 + T_second/2 bis 3/4*T_0 + T_first/2 + T_second wird der RZ Z_second eingeschaltet
        if(t_zeitpunkt>(3*T_0/4+T_first/2+T_second/2) && t_zeitpunkt<= (3*T_0/4 + T_first/2+T_second))
        {pulse[0] = Z[Z_second][0]; pulse[1] = Z[Z_second][1]; pulse[2]=Z[Z_second][2];}
        
        //für die Zeit 3/4*T_0 + T_first/2 + T_second bis 3/4*T_0 + T_first + T_second wird der RZ Z_first eingeschaltet
        if(t_zeitpunkt>(3*T_0/4+T_first/2+T_second) && t_zeitpunkt<= (3*T_0/4 + T_first+T_second))
        {pulse[0] = Z[Z_first][0]; pulse[1] = Z[Z_first][1]; pulse[2]=Z[Z_first][2];}
        
        // für die Zeit 3/4*T_0 + T_first + T_second bis 1/f_takt wird der RZ Z0 geschaltet
        if(t_zeitpunkt>(3*T_0/4+T_first+T_second) && t_zeitpunkt<= N)
        {pulse[0] = Z[0][0]; pulse[1] = Z[0][1]; pulse[2]=Z[0][2];}
        
      //Ausgabe der Variablen   
        out_svm[0] = pulse[0];
        out_svm[1] = pulse[1];
        out_svm[2] = pulse[2];
     
        //Erhöhunh des Schrittzählers
        t_zeitpunkt ++;
}
