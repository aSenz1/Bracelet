import dash
from dash import dcc, html, Input, Output, State, no_update
import serial
import serial.tools.list_ports
from time import sleep

app = dash.Dash(__name__)

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [{'label': f"{port.device} - {port.description}", 'value': port.device} for port in ports]

app.layout = html.Div([
    html.H1("Controle Arduino via Serial", style={'textAlign': 'center'}),
    
    html.Div([
        html.Div([
            html.Label("Porta Serial:", style={'fontWeight': 'bold'}),
            dcc.Dropdown(
                id='serial-port',
                options=list_serial_ports(),
                placeholder="Selecione a porta serial",
                style={'width': '100%'}
            ),
        ], style={'margin': '10px', 'flex': 1}),
        
        html.Div([
            html.Label("Baud Rate:", style={'fontWeight': 'bold'}),
            dcc.Input(
                id='baud-rate',
                type='number',
                value=9600,
                placeholder="Digite o baud rate",
                style={'width': '100%'}
            ),
        ], style={'margin': '10px', 'flex': 1}),
    ], style={'display': 'flex', 'flexDirection': 'row'}),
    
    html.Div([
        html.Button('Conectar', id='connect-btn', n_clicks=0, 
                   style={'padding': '10px', 'fontSize': '16px'}),
        html.Div(id='connection-status', style={'marginLeft': '10px'})
    ], style={'margin': '20px', 'display': 'flex', 'alignItems': 'center'}),
    
    html.Div([
        html.Label("Valor para enviar:", style={'fontWeight': 'bold'}),
        dcc.Input(
            id='input-value',
            type='text',
            placeholder="Digite o valor (0 para desligar, 1 para ligar)",
            style={'width': '100%'}
        ),
        html.Button('Enviar', id='send-btn', n_clicks=0, disabled=True,
                  style={'marginTop': '10px', 'padding': '10px', 'fontSize': '16px'}),
    ], style={'margin': '20px'}),
    
    html.Div([
        html.H3("Respostas do Arduino:"),
        html.Div(id='arduino-responses', 
                style={
                    'border': '1px solid #ddd',
                    'padding': '10px',
                    'minHeight': '100px',
                    'maxHeight': '300px',
                    'overflowY': 'auto',
                    'backgroundColor': '#f9f9f9'
                })
    ], style={'margin': '20px'}),
    
    dcc.Interval(id='response-updater', interval=1000, disabled=True),
    
    html.Div(id='output-message', style={'margin': '20px', 'fontWeight': 'bold'})
])

serial_connection = None
arduino_responses = []

@app.callback(
    [Output('connection-status', 'children'),
     Output('connect-btn', 'children'),
     Output('send-btn', 'disabled'),
     Output('response-updater', 'disabled')],
    [Input('connect-btn', 'n_clicks')],
    [State('serial-port', 'value'),
     State('baud-rate', 'value')],
    prevent_initial_call=True
)
def connect_serial(n_clicks, port, baud_rate):
    global serial_connection, arduino_responses
    if n_clicks % 2 == 1:  # Conectar
        try:
            serial_connection = serial.Serial(port, baud_rate, timeout=1)
            sleep(2)  # Espera a conexão estabilizar
            arduino_responses = []  # Limpa respostas anteriores
            return [f"Conectado a {port} @ {baud_rate} baud", 
                   "Desconectar", 
                   False,
                   False]  # Ativa atualizador
        except Exception as e:
            return [f"Erro: {str(e)}", "Conectar", True, True]
    else:  # Desconectar
        if serial_connection and serial_connection.is_open:
            serial_connection.close()
        return ["Desconectado", "Conectar", True, True]

@app.callback(
    [Output('output-message', 'children'),
     Output('input-value', 'value')],
    [Input('send-btn', 'n_clicks')],
    [State('input-value', 'value'),
     State('serial-port', 'value')],
    prevent_initial_call=True
)
def send_value(n_clicks, value, port):
    global serial_connection
    if not n_clicks or value is None:
        return no_update, no_update
    
    if not serial_connection or not serial_connection.is_open:
        return "Erro: Não conectado ao Arduino", no_update
    
    try:
        serial_connection.write(f"{value}\n".encode())
        return f"Valor {value} enviado com sucesso!", ''
    except Exception as e:
        return f"Erro ao enviar: {str(e)}", no_update

@app.callback(
    Output('arduino-responses', 'children'),
    [Input('response-updater', 'n_intervals')],
    prevent_initial_call=True
)
def update_responses(n):
    global serial_connection, arduino_responses
    if not serial_connection or not serial_connection.is_open:
        return no_update
    
    try:
        while serial_connection.in_waiting:
            response = serial_connection.readline().decode().strip()
            if response:
                arduino_responses.append(response)
                if len(arduino_responses) > 10:  # Limita o histórico
                    arduino_responses.pop(0)
    except:
        pass
    
    return [html.Div(resp) for resp in arduino_responses]

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
