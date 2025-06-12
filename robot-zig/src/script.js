const pidFormUpright = document.getElementById('pidFormUpright');

pidFormUpright.addEventListener('submit', function(event) {
    event.preventDefault();
    const formData = new FormData(pidFormUpright);
    const data = {};
    formData.forEach((value, key) => {
        data[key] = value;
    });

    fetch('/pidUpright', {
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

const pidFormFalling = document.getElementById('pidFormFalling');

pidFormFalling.addEventListener('submit', function(event) {
    event.preventDefault();
    const formData = new FormData(pidFormFalling);
    const data = {};
    formData.forEach((value, key) => {
        data[key] = value;
    });

    fetch('/pidFalling', {
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

const speakForm = document.getElementById('speakForm');

speakForm.addEventListener('submit', function(event) {
    event.preventDefault();
    const formData = new FormData(speakForm);
    const data = {};
    formData.forEach((value, key) => {
        data[key] = value;
    });

    fetch('/speak', {
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