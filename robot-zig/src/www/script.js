// File: robot-zig/www/script.js

bindPIDForm('Stable');
bindPIDForm('Fine');
bindPIDForm('Moderate');
bindPIDForm('Falling');

function bindPIDForm(state) {
    const pidForm = document.getElementById('pidForm'+state);
    pidForm.addEventListener('submit', function(event) {
        event.preventDefault();
        const formData = new FormData(pidFormFalling);
        const data = {};
        formData.forEach((value, key) => {
            data[key] = Number(value) || value;
        });

        fetch('/api/pid'+state, {
            method: 'POST',
            headers: {
            'Content-Type': 'application/json'
            },
            body: JSON.stringify(data)
        })
        .then(response => response.json())
        .then(data => {
            console.log('Success:', data);
        })
        .catch(error => {
            console.error('Error:', error);
        });
    });
}

const speakForm = document.getElementById('speakForm');

speakForm.addEventListener('submit', function(event) {
    event.preventDefault();
    const formData = new FormData(speakForm);
    const data = {};
    formData.forEach((value, key) => {
        data[key] = value;
    });

    fetch('/api/speak', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(data)
    })
    .then(response => response.json())
    .then(data => {
        console.log('Success:', data);
        // TODO set curKp, curKi, curKd to the values in data (Kp=####, ki=####, Kd=####)
    })
    .catch(error => {
        console.error('Error:', error);
    });
});