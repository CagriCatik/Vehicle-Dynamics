# Vertikaldynamik: Verifikation und Straßenanregung (M8)

Module:
- [`suspension/vertical_analysis.py`](../src/vehicle_model/suspension/vertical_analysis.py): analytische Referenzen.
- [`roads/road_profile.py`](../src/vehicle_model/roads/road_profile.py): Straßenprofile.
- [`suspension/quarter_car.py`](../src/vehicle_model/suspension/quarter_car.py): `step_with_road`.

Die Modelle bestanden schon vor M8: Viertelfahrzeug, 14-DOF-Modell, Skyhook/Groundhook, aktive Wankstabilisierung (siehe [SUSPENSION_DYNAMICS.md](SUSPENSION_DYNAMICS.md)). Die bisherigen Tests prüften nur Plausibilitätsbereiche, etwa „Aufbaueigenfrequenz zwischen 1 und 2 Hz“. M8 ergänzt die Abnahme: **Freie Schwingungen, Energieverhalten, Frequenzgänge und definierte Straßenanregungen werden korrekt berechnet.**

---

## 1. Analytische Referenzen (lineares Viertelfahrzeug)

Koordinaten ab statischer Ruhelage, Zustand $x = [z_s, \dot z_s, z_u, \dot z_u]$, Eingang Straßenhöhe $z_r$ und -geschwindigkeit $\dot z_r$.

- **Moden:** Eigenwerte der Systemmatrix. Ohne Dämpfung sind die Eigenfrequenzen die Wurzeln von $m_s m_u \omega^4 - (k_s(m_s + m_u) + k_t m_s)\omega^2 + k_s k_t = 0$.
- **Frequenzgänge** je Einheit Straßenamplitude: $H_x(j\omega) = (j\omega I - A)^{-1}(B_z + j\omega B_v)$. Daraus folgen Aufbaubeschleunigung $-\omega^2 H_{z_s}$, Federweg $H_{z_s} - H_{z_u}$ und dynamische Radlast $-(k_t + c_t j\omega)(H_{z_u} - 1)$.
- **Energie:** $E = \tfrac12 m_s\dot z_s^2 + \tfrac12 m_u\dot z_u^2 + \tfrac12 k_s(z_s - z_u)^2 + \tfrac12 k_t(z_u - z_r)^2$. Die Schwerkraft entfällt durch die statischen Vorspannungen.
- **Leistungsbilanz:** $\dot E = -c_s(\dot z_s - \dot z_u)^2 - c_t(\dot z_u - \dot z_r)^2 + P_\text{Straße}$ mit $P_\text{Straße} = -[k_t(z_u - z_r) + c_t(\dot z_u - \dot z_r)]\,\dot z_r$ (hergeleitet und am Modell geprüft).

Die Referenzen gelten, solange der Reifen Kontakt hat und die Anschläge nicht erreicht sind. Das nichtlineare Modell löst den Kontakt (M2).

## 2. Befund: Straßenanregung im Integrationsschritt

`QuarterCarModel.step` hält Straßenhöhe und -geschwindigkeit über den ganzen RK4-Schritt konstant (Halteglied nullter Ordnung). Die Anregung eilt dadurch um $dt/2$ nach, und die Energiebilanz ist nur erster Ordnung genau. Bei $dt$ = 10⁻⁴ s und 4 Hz Anregung fehlen 15 %; der Fehler halbiert sich mit halbem Schritt.

Neu ist `step_with_road(state, t, dt, road)`. Es wertet die Straße als Zeitfunktion an den RK4-Stützstellen aus. Damit schließt die Energiebilanz auf 4·10⁻⁶. `step` bleibt für bestehende Aufrufer unverändert.

## 3. Straßenprofile

**ISO 8608:** einseitige Unebenheitsdichte $G_d(n) = G_d(n_0)(n/n_0)^{-2}$, $n_0$ = 0,1 Zyklen/m. Klassenwerte als geometrische Mittel: A = 16·10⁻⁶ m³, je Klasse ×4, bis H. Das Profil entsteht durch harmonische Überlagerung mit geseedeten Zufallsphasen, Amplitude $\sqrt{2 G_d(n_i)\Delta n}$; die Dichte stimmt an den Stützfrequenzen damit exakt. Bei Geschwindigkeit $v$ gilt $z_r(t) = z(x_0 + vt)$ und $\dot z_r = v\,dz/dx$.

**Einzelhindernis:** Halbsinus-Schwelle mit Höhe und Länge, stetige Steigung.

Die Profile sind synthetische Prüfanregungen, keine gemessenen Straßen.

## 4. Verifikation

[test_vertical_dynamics.py](../tests/vehicle_model/test_vertical_dynamics.py), Standardparameter des Viertelfahrzeugs: Aufbau 1,45 Hz bei ζ = 0,24, Radeigenfrequenz 11,5 Hz bei ζ = 0,39.

| Prüfung | Ergebnis |
|---|---|
| Ungedämpfte Eigenfrequenzen = Wurzeln der charakteristischen Gleichung | rel. 1e-10 |
| Freie Schwingung, ungedämpft: Spektrumsspitzen = Eigenfrequenzen | innerhalb der Auflösung 1/T |
| Energieerhaltung ohne Dämpfung, 40 000 RK4-Schritte | 1e-6 |
| Gedämpfte freie Schwingung: Energie nimmt monoton ab; abgegebene Energie = Dämpferarbeit | 1e-3 |
| Simulierter gegen analytischen Frequenzgang (Aufbaubeschleunigung, Federweg, dynamische Radlast), 1–12 Hz | ≤ 1 % (im Beispiel max. 0,26 %) |
| Energiebilanz mit Straßenanregung | 1e-4 |
| ISO 8608: Klassenskalierung, Effektivwert, Welch-Spektrum gegen Zieldichte | Median 1 ± 0,1 |
| Steigung = Ableitung der Höhe (ISO-Profil, Schwelle) | 1e-4 |
| Hohe Schwelle: Rad hebt ab, Kontaktverlust gemeldet, Radlast ≥ 0 | erfüllt |

**Semiaktive Dämpfung**, gemessen gegen den passiven Dämpfer (2200 N s/m):

| | Aufbaubeschleunigung, 1,45 Hz | dyn. Radlast, 11,5 Hz | dyn. Radlast, 1,45 Hz | ISO C, 72 km/h: Aufbaubeschleunigung / dyn. Radlast |
|---|---|---|---|---|
| Skyhook | **−33 %** | +76 % | −32 % | **−18 %** / +10 % |
| Groundhook | +116 % | **−2,6 %** | +112 % | +13 % / **+8 %** |

Skyhook verbessert den Komfort und bezahlt das mit Radlastschwankungen nahe der Radeigenfrequenz. Groundhook hilft bei der Standardabstimmung nur nahe der Radeigenfrequenz geringfügig. Auf breitbandiger Anregung ist er schlechter als der passive Dämpfer, weil die Aufbaumode dann mit $c_{min}$ = 800 N s/m kaum gedämpft ist. Der Test prüft diesen Zielkonflikt so, wie er ist. Die frühere Doku-Aussage „maximiert den Kraftschluss“ ist korrigiert, ebenso die Skyhook-Angabe „> 20 %“, die nicht belegt war.

## 5. Beispiel und Grenzen

`python examples/vertical_dynamics_demo.py` erzeugt `output/vertical_dynamics/vertical_dynamics.png`: Frequenzgänge passiv (analytisch und simuliert), Skyhook und Groundhook simuliert, ISO-Klassen A–E mit dem Spektrum eines erzeugten Klasse-C-Profils.

**Grenzen:**
- Halbfahrzeugmodell nicht umgesetzt (laut Plan „nach Bedarf“); Heben, Wanken und Nicken bildet das 14-DOF-Modell ab.
- Die Kopplung mit der Planardynamik besteht nur im 14-DOF-Modell. Das 7-DOF-Modell bleibt quasistatisch (M4).
- Der Groundhook ist nicht nachoptimiert.
- Keine ISO-2631-Komfortbewertung (frequenzgewichtete Beschleunigung).
